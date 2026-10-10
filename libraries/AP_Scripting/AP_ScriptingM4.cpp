#include "AP_ScriptingM4.h"

#if AP_SCRIPTING_M4_ENABLED

#include <errno.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include "AP_ScriptingM4_Layout.h"
#include "AP_ScriptingM4_Protocol.h"

extern const AP_HAL::HAL& hal;

namespace PeerHeader = CubePilotFW::PeerHeader;

// one wake serves at most this many requests or this long (spec §7)
static const uint8_t MAX_REQUESTS_PER_WAKE = 32;
static const uint32_t MAX_SERVE_US = 200;
// protocol errors within one second that count as a failure
static const uint16_t PROTOCOL_ERROR_LIMIT = 3;

AP_ScriptingM4 *AP_ScriptingM4::_singleton;

const AP_Param::GroupInfo AP_ScriptingM4::var_info[] = {
    // @Param: RST_MAX
    // @DisplayName: Scripting M4 restart limit
    // @Description: Number of times the M4 scripting core is restarted after a fault before scripting turns off until the next reboot
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("RST_MAX", 1, AP_ScriptingM4, _restart_max, 3),

#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    // @Param: INJECT
    // @DisplayName: Scripting M4 fault injection
    // @Description: Test only. Changing this to a non-zero value makes the M4 scripting core fault in that way
    // @Values: 0:None,1:HardFault,2:MemManage,3:Stall,4:Lockup,5:Ring corruption
    // @User: Advanced
    AP_GROUPINFO("INJECT", 2, AP_ScriptingM4, _inject, 0),
#endif

    AP_GROUPEND
};

AP_ScriptingM4::AP_ScriptingM4()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_ScriptingM4::init()
{
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    _inject_seen = _inject.get();   // a value saved before this boot is not a request
#endif
    const uint32_t now_ms = AP_HAL::millis();
    _stats_start_ms = now_ms;
    _channel = NEW_NOTHROW CubePilotFW::Channel();
    if (_channel == nullptr) {
        go_off("out of memory");
        return;
    }
    char reason[40] {};
    if (!platform_start(reason, sizeof(reason))) {
        go_off(reason);
        return;
    }
    enter(State::Starting, now_ms);
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ScriptingM4::host_thread, void),
                                      "ScriptingM4", 4096, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0)) {
        go_off("no host thread");
    }
}

void AP_ScriptingM4::doorbell_isr(uint8_t group)
{
    (void)group;
    if (_singleton != nullptr) {
        _singleton->_doorbell_us = AP_HAL::micros();
        _singleton->_doorbell.signal_ISR();
    }
}

void AP_ScriptingM4::host_thread()
{
    bool serving = false;   // the last serve() left frames it can take on the next pass
    while (true) {
        if (_state == State::Off) {
            platform_m4_exited();   // SITL: reap the stopped process
            const uint32_t off_ms = AP_HAL::millis();
            if (off_ms - _stats_start_ms >= 1000) {
                log_supervisor(off_ms, off_ms - _stats_start_ms);
                _stats_start_ms = off_ms;
            }
            hal.scheduler->delay(100);
            continue;
        }
        // a ring that cannot be served now (not running, or a frame stuck after
        // a protocol error) must not make this loop spin: wait for the timeout
        if (_channel->prepareSleep() || !serving) {
            IGNORE_RETURN(_doorbell.wait(platform_wait_us()));
        }
        _channel->wake();
        _woke_us = AP_HAL::micros();
        const uint32_t doorbell_us = _doorbell_us;
        _doorbell_us = 0;
        _wake_latency_us = doorbell_us != 0 ? _woke_us - doorbell_us : 0;
        serving = serve();
        const uint32_t now_ms = AP_HAL::millis();
        PeerHeader::beat(_shared, PeerHeader::Core::M7);
        supervise(now_ms);
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
        send_fault_inject();
#endif
        _host_busy_us += AP_HAL::micros() - _woke_us;
        if (now_ms - _stats_start_ms >= 1000) {
            log_supervisor(now_ms, now_ms - _stats_start_ms);
            _stats_start_ms = now_ms;
            _requests = 0;
            _protocol_errors = 0;
            _dropped = 0;
            _host_busy_us = 0;
        }
    }
}

// serve a bounded batch of M4 requests; false if the ring cannot be served now
bool AP_ScriptingM4::serve()
{
    if (_state != State::Running) {
        return false;   // frames from before a (re)start are dropped by the re-handshake
    }
    for (uint8_t i = 0; i < MAX_REQUESTS_PER_WAKE; i++) {
        if (AP_HAL::micros() - _woke_us > MAX_SERVE_US) {
            return true;
        }
        uint8_t buf[ScriptingM4::MAX_MESSAGE];
        uint16_t len = 0;
        const int ret = _channel->receive(buf, sizeof(buf), &len);
        if (ret == -EAGAIN) {
            return true;
        }
        if (ret != 0) {
            // -EBADMSG: corrupt or desynchronised ring; -EMSGSIZE: an oversized frame stays queued
            _protocol_errors++;
            return false;
        }
        handle_message(buf, len);
    }
    return true;
}

void AP_ScriptingM4::handle_message(const uint8_t *buf, uint16_t len)
{
    const uint32_t start_us = AP_HAL::micros();
    ScriptingM4::MsgHeader header;
    if (!ScriptingM4::parse_header(buf, len, header)) {
        _protocol_errors++;
        return;
    }
    _requests++;
    switch (ScriptingM4::MsgType(header.type)) {
    case ScriptingM4::MsgType::Ping: {
        ScriptingM4::PongMsg pong;
        ScriptingM4::fill_header(pong.header, ScriptingM4::MsgType::Pong, sizeof(pong), header.seq);
        pong.m7_wake_us = _wake_latency_us;
        pong.m7_exec_us = AP_HAL::micros() - start_us;
        if (_channel->send(&pong, sizeof(pong)) != 0) {
            _dropped++;
        }
        return;
    }
    case ScriptingM4::MsgType::Stats:
        log_rpc_stats(buf);
        return;
    case ScriptingM4::MsgType::Pong:
    case ScriptingM4::MsgType::FaultInject:
    case ScriptingM4::MsgType::Call:
    case ScriptingM4::MsgType::Reply:
    case ScriptingM4::MsgType::Notify:
    case ScriptingM4::MsgType::Ready:
        _protocol_errors++;   // M7 -> M4 only, or RPC frames not served yet
        return;
    }
}

// SRPC as in AP_Scripting_BindingStats.cpp; SemMax carries the largest M7-side time of one request
void AP_ScriptingM4::log_rpc_stats(const uint8_t *buf)
{
    ScriptingM4::StatsMsg stats;
    memcpy(&stats, buf, sizeof(stats));
    const uint32_t now_ms = AP_HAL::millis();
    _m4_round_trips.count = stats.count;
    _m4_round_trips.p50_us = stats.p50_us;
    _m4_round_trips.p99_us = stats.p99_us;
    _m4_round_trips.max_us = stats.max_us;
    // the M4 keeps a window it could not send, so a report may cover more than a second
    _m4_round_trips.window_ms = _m4_stats_ms != 0 ? now_ms - _m4_stats_ms : 1000;
    _m4_stats_ms = now_ms;
    _m4_pings_lost += stats.lost;
#if HAL_LOGGING_ENABLED && AP_SCRIPTING_RPC_STATS_ENABLED
    AP::logger().WriteStreaming("SRPC", "TimeUS,Id,Name,N,P50,P99,Max,SemMax,Tot",
                                "s#--sssss", "F---FFFFF", "QHZIIIIIQ",
                                AP_HAL::micros64(), uint16_t(0), "m4:ping", stats.count,
                                stats.p50_us, stats.p99_us, stats.max_us, stats.m7_max_us,
                                uint64_t(stats.total_us));
#endif
}

// @LoggerMessage: M4ST
// @Description: Scripting M4 supervisor, once per second (M4 scripting builds)
// @Field: TimeUS: Time since system startup
// @Field: St: supervisor state (0 held, 1 starting, 2 running, 3 restarting, 4 off)
// @Field: HB: M4 heartbeat counter
// @Field: Ses: M4 peer header session
// @Field: Rst: restarts used
// @Field: Flt: M4 fault count
// @Field: FPC: PC of the last M4 fault
// @Field: Req: M4 messages served in the last second
// @Field: PErr: protocol errors in the last second
// @Field: Drop: replies dropped because the M4's ring was full
// @Field: Lost: M4 pings without a reply since the previous row
void AP_ScriptingM4::log_supervisor(uint32_t now_ms, uint32_t elapsed_ms)
{
#if HAL_LOGGING_ENABLED && AP_SCRIPTING_RPC_STATS_ENABLED
    AP::logger().WriteStreaming("M4ST", "TimeUS,St,HB,Ses,Rst,Flt,FPC,Req,PErr,Drop,Lost",
                                "s----------", "F----------", "QBIIBIIIHHI",
                                AP_HAL::micros64(), uint8_t(_state), _m4_heartbeat, _m4_session,
                                _restarts_used, _m4_fault_count, _m4_fault_pc, _requests,
                                _protocol_errors, _dropped, _m4_pings_lost);
#endif
    _m4_pings_lost = 0;
    if (!_summary_enabled || _state != State::Running || now_ms - _last_summary_ms < 10000) {
        return;
    }
    _last_summary_ms = now_ms;
    const uint32_t per_second = uint32_t(uint64_t(_m4_round_trips.count) * 1000 / MAX(_m4_round_trips.window_ms, 1U));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting RPC %u/s p50<%u p99<%u max %uus",
                  unsigned(per_second), unsigned(_m4_round_trips.p50_us),
                  unsigned(_m4_round_trips.p99_us), unsigned(_m4_round_trips.max_us));
    // busy microseconds per millisecond = per mille of one core
    const uint32_t host_permille = _host_busy_us / MAX(elapsed_ms, 1U);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting M7 host %u.%u%% CPU",
                  unsigned(host_permille / 10), unsigned(host_permille % 10));
}

void AP_ScriptingM4::supervise(uint32_t now_ms)
{
    switch (_state) {
    case State::Starting:
    case State::Restarting:
        check_started(now_ms);
        break;
    case State::Running:
        check_running(now_ms);
        break;
    case State::Held:
    case State::Off:
        break;
    }
}

void AP_ScriptingM4::check_started(uint32_t now_ms)
{
    PeerHeader::View view {};
    const PeerHeader::Status status = PeerHeader::readPeer(_shared, PeerHeader::Core::M7,
                                                           ScriptingM4::PROTOCOL_VERSION,
                                                           ScriptingM4::CONTRACT_HASH, &view);
    if (status == PeerHeader::Status::Mismatch) {
        go_off("image mismatch");
        return;
    }
    if (status == PeerHeader::Status::Compatible && view.restarts == _restarts_used &&
        (_state == State::Starting || view.session != _m4_session)) {
        _m4_session = view.session;
        _m4_fault_count = view.faultCount;
        _m4_heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
        _m4_heartbeat_ms = now_ms;
        // re-handshake: drop whatever either side queued before this start
        _channel->resetAfterRestart();
        _m4_stats_ms = 0;
        if (_state == State::Starting) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: M4 started");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: M4 restarted (%u/%u)",
                          unsigned(_restarts_used), unsigned(restart_limit()));
        }
        enter(State::Running, now_ms);
        return;
    }
    const uint32_t timeout_ms = _state == State::Starting ? ScriptingM4::START_TIMEOUT_MS
                                                          : ScriptingM4::RESTART_TIMEOUT_MS;
    if (now_ms - _state_start_ms > timeout_ms) {
        go_off("not responding");
    }
}

void AP_ScriptingM4::check_running(uint32_t now_ms)
{
    if (platform_m4_exited()) {
        failure("process exit");
        return;
    }
    PeerHeader::View view {};
    const PeerHeader::Status status = PeerHeader::readPeer(_shared, PeerHeader::Core::M7,
                                                           ScriptingM4::PROTOCOL_VERSION,
                                                           ScriptingM4::CONTRACT_HASH, &view);
    if (status == PeerHeader::Status::Mismatch) {
        go_off("image mismatch");
        return;
    }
    if (status == PeerHeader::Status::Compatible) {
        if (view.faultCount != _m4_fault_count) {
            _m4_fault_count = view.faultCount;
            _m4_fault_pc = view.faultPc;
            char reason[40];
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            hal.util->snprintf(reason, sizeof(reason), "signal %u addr=0x%08X",
                               unsigned(view.faultCfsr), unsigned(view.faultPc));
#else
            hal.util->snprintf(reason, sizeof(reason), "%s PC=0x%08X",
                               ScriptingM4::fault_name(view.faultCfsr, view.faultHfsr), unsigned(view.faultPc));
#endif
            failure(reason);
            return;
        }
        if (view.session != _m4_session) {
            failure("restarted unasked");
            return;
        }
    }
    // Busy: the M4 is rewriting its header (or died doing so); the heartbeat decides
    const uint32_t heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
    if (heartbeat != _m4_heartbeat) {
        _m4_heartbeat = heartbeat;
        _m4_heartbeat_ms = now_ms;
    } else if (now_ms - _m4_heartbeat_ms > ScriptingM4::HEARTBEAT_TIMEOUT_MS) {
        failure("heartbeat lost");
        return;
    }
    if (_protocol_errors >= PROTOCOL_ERROR_LIMIT) {
        failure("protocol errors");
    }
}

uint8_t AP_ScriptingM4::restart_limit() const
{
    return uint8_t(constrain_int16(_restart_max.get(), 0, 10));
}

void AP_ScriptingM4::failure(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: M4 %s", reason);
    if (_restarts_used >= restart_limit()) {
        go_off("fault limit");
        return;
    }
    _restarts_used++;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Scripting: M4 restarting (%u/%u)",
                  unsigned(_restarts_used), unsigned(restart_limit()));
    _protocol_errors = 0;
    // the M4 acts on a restart doorbell only once the grant has changed: store it first
    ScriptingM4::supervisor_block(_shared)->restart_grant.store(_restarts_used);
    platform_request_restart();
    enter(State::Restarting, AP_HAL::millis());
}

void AP_ScriptingM4::go_off(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: M4 %s, off until reboot", reason);
    if (_shared != nullptr) {
        ScriptingM4::supervisor_block(_shared)->run_permit.store(0);
    }
    // after run_permit: stopping rings the restart doorbell, which the M4 reads as park
    platform_stop();
    enter(State::Off, AP_HAL::millis());
}

void AP_ScriptingM4::enter(State state, uint32_t now_ms)
{
    _state = state;
    _state_start_ms = now_ms;
}

bool AP_ScriptingM4::arming_checks(size_t buflen, char *buffer) const
{
    if (_state == State::Off || _state == State::Held) {
        hal.util->snprintf(buffer, buflen, "Scripting core not running");
        return false;
    }
    return true;
}

#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
void AP_ScriptingM4::send_fault_inject()
{
    const int8_t kind = _inject.get();
    if (kind == _inject_seen) {
        return;
    }
    _inject_seen = kind;
    if (kind <= 0 || kind > ScriptingM4::FAULT_KIND_MAX || _state != State::Running) {
        return;
    }
    ScriptingM4::FaultInjectMsg inject;
    ScriptingM4::fill_header(inject.header, ScriptingM4::MsgType::FaultInject, sizeof(inject), 0);
    inject.kind = uint32_t(kind);
    if (_channel->send(&inject, sizeof(inject)) != 0) {
        _dropped++;
    }
}
#endif

#endif  // AP_SCRIPTING_M4_ENABLED
