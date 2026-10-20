#include "AP_RustM4.h"

#if AP_RUST_M4_ENABLED

#include <errno.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include "AP_RustM4_Layout.h"

static_assert(HAL_WITH_EKF_DOUBLE && HAL_WITH_POSTYPE_DOUBLE,
              "the M4 computes Location math in double (m4/shim/AP_HAL/AP_HAL_Boards.h)");

extern const AP_HAL::HAL& hal;

namespace PeerHeader = CubePilotFW::PeerHeader;
using RustM4::MsgType;

// after a pass that hit either limit with frames left: a flooding M4 gets one pass per pause
static const uint32_t SERVE_BACKOFF_US = 500;
// a pass this long in wall time is counted (M4ST.Lng): blocked in a binding, or preempted
static const uint32_t LONG_PASS_US = 50000;
// protocol errors within one second that count as a failure
static const uint16_t PROTOCOL_ERROR_LIMIT = 3;
// Ready is repeated this often until the first call of a session
static const uint32_t READY_REPEAT_MS = 1000;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// a SITL restart waits on host process scheduling (kill, execv, dynamic loading), not on M4 boot time
static const uint32_t PLATFORM_RESTART_TIMEOUT_MS = RustM4::START_TIMEOUT_MS;
#else
static const uint32_t PLATFORM_RESTART_TIMEOUT_MS = RustM4::RESTART_TIMEOUT_MS;
#endif
// RUST_DEBUG_OPTS bit that disables the Rust pre-arm checks (AP_Rust::Runtime)
static const uint8_t DEBUG_OPTION_NO_PREARM = 1U << 4;

AP_RustM4 *AP_RustM4::_singleton;

const AP_Param::GroupInfo AP_RustM4::var_info[] = {
    // @Param: RST_MAX
    // @DisplayName: Rust M4 restart limit
    // @Description: M4 restarts after faults before native modules stay off until reboot
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("RST_MAX", 1, AP_RustM4, _restart_max, 3),
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
    // @Param: INJECT
    // @DisplayName: Rust M4 fault injection
    // @Description: Test builds only: performs this fault kind on the M4 once when set
    // @User: Advanced
    AP_GROUPINFO("INJECT", 2, AP_RustM4, _inject, 0),
#endif
    // @Param: OPTS
    // @DisplayName: Rust M4 options
    // @Description: Rust on the Cortex-M4 options
    // @Bitmask: 0:GCS RPC summary every 10 s,1:GCS M4 CRC self-check values once per session
    // @User: Advanced
    AP_GROUPINFO("OPTS", 3, AP_RustM4, _options, 0),
    AP_GROUPEND
};

AP_RustM4::AP_RustM4()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_RustM4::init(const AP_Rust::Runtime::Config &config)
{
    if (_initialized) {
        return;
    }
    _initialized = true;
    _observed_restart = config.restart;
    _prearm_disabled = (uint8_t(config.debug_options) & DEBUG_OPTION_NO_PREARM) != 0;
    if (config.enable == 0) {
        return;   // Held: the M4 stays in reset (RUST_ENABLE needs a reboot)
    }
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
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
    // binding calls will run vehicle code on this thread. Above the I/O-class threads (io 58,
    // storage/log_io 59, UART/net 60) so a call is not queued behind logging; below every
    // vehicle-timing thread (I2C 176 and up). The duty cap bounds the CPU share a flooding M4
    // can take from those threads
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_RustM4::host_thread, void),
                                      "RustM4", 8192, AP_HAL::Scheduler::PRIORITY_UART,
                                      RustM4::HOST_PRIORITY_OFFSET)) {
        go_off("no host thread");
    }
}

void AP_RustM4::update(const AP_Rust::Runtime::Config &config)
{
    _prearm_disabled = (uint8_t(config.debug_options) & DEBUG_OPTION_NO_PREARM) != 0;
    if (!_initialized) {
        return;
    }
    // RUST_RESTART is a counter: any change asks for a restart of the modules, which on the M4 is
    // a requested M4 restart (no fault, none of RUST_M4_RST_MAX spent)
    if (config.restart != _observed_restart) {
        _observed_restart = config.restart;
        _run_requests.restart();
    }
}

void AP_RustM4::doorbell_isr(uint8_t group)
{
    (void)group;
    if (_singleton != nullptr) {
        _singleton->_doorbell_us = AP_HAL::micros();
        _singleton->_doorbell.signal_ISR();
    }
}

void AP_RustM4::host_thread()
{
    ServeResult last = ServeResult::Idle;
    while (true) {
        if (_state == State::Off) {
            platform_m4_exited();   // SITL: reap the stopped process
            const uint32_t off_ms = AP_HAL::millis();
            if (off_ms - _stats_start_ms >= 1000) {
                log_supervisor(off_ms, off_ms - _stats_start_ms);
                _stats_start_ms = off_ms;
                _host_busy_us = 0;
                _host_cpu_us = 0;
                _pass_cpu_max_us = 0;
            }
            hal.scheduler->delay(100);
            continue;
        }
        const uint32_t cap_wait_us = _duty_cap.wait_us(AP_HAL::micros());
        const bool ring_empty = _channel->prepareSleep();   // sets the asleep flag: the M4 rings from here on
        switch (RustM4::host_wait_after(cap_wait_us, last == ServeResult::Busy, last == ServeResult::Stuck,
                                        ring_empty)) {
        case RustM4::HostWait::DutyCap:
            // the host's share of the M7 ran out: serve nothing until credit returns, however many
            // frames are pending. An empty ring keeps listening, so its next doorbell is not missed
            _duty_cap_hits++;
            pause_for_duty_cap(MIN(cap_wait_us, platform_wait_us()), ring_empty);
            // frames pending after the pause are served at once (the cap still comes first)
            last = ServeResult::Idle;
            break;
        case RustM4::HostWait::Backoff:
            // the budget ran out with frames left: pause so a flooding M4 costs bounded M7 time.
            // No doorbell is needed while we pause
            _channel->wake();
            hal.scheduler->delay_microseconds(SERVE_BACKOFF_US);
            break;
        case RustM4::HostWait::Doorbell:
        case RustM4::HostWait::Timeout:
            // a ring that cannot be served now (not running, or a frame stuck after
            // a protocol error) must not make this loop spin: wait for the timeout
            if (ring_empty || last == ServeResult::Stuck) {
                IGNORE_RETURN(_doorbell.wait(platform_wait_us()));
            }
            break;
        }
        _channel->wake();
        _woke_us = AP_HAL::micros();
        const uint32_t doorbell_us = _doorbell_us;
        _doorbell_us = 0;
        _wake_latency_us = doorbell_us != 0 ? _woke_us - doorbell_us : 0;
        // the only call of serve(): a pause shorter than the cap's wait (the supervisor still
        // runs every platform_wait_us()) serves nothing. ChibiOS: the wait, the wake and the
        // switch before this pass are charged after the decision
        if (RustM4::pass_after_wake(_duty_cap, _woke_us, [this]() { charge_cpu(false); })) {
            last = serve();
        }
        const uint32_t now_ms = AP_HAL::millis();
        PeerHeader::beat(_shared, PeerHeader::Core::M7);
        supervise(now_ms);
        if (_state == State::Running) {
            // after this pass's frames: a restart never makes the host thread wait
            run_control(now_ms);
        }
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
        send_fault_inject();
#endif
        const uint32_t busy_us = AP_HAL::micros() - _woke_us;
        _host_busy_us += busy_us;
        if (busy_us > LONG_PASS_US) {
            _long_passes++;
        }
        charge_cpu(true);
        if (now_ms - _stats_start_ms >= 1000) {
            log_supervisor(now_ms, now_ms - _stats_start_ms);
            _stats_start_ms = now_ms;
            _requests = 0;
            _protocol_errors = 0;
            _dropped = 0;
            _host_busy_us = 0;
            _duty_cap_hits = 0;
            _host_cpu_us = 0;
            _long_passes = 0;
            _pass_cpu_max_us = 0;
        }
    }
}

// charge the duty cap the host's CPU since its previous reading (HostCpuCharge); at a pass end
// that is the pass, the largest of which M4HP logs
void AP_RustM4::charge_cpu(bool pass_end)
{
    uint64_t cpu_us;
    const bool reset = platform_thread_cpu_us(cpu_us);
    const uint32_t wall_us = AP_HAL::micros();
    const uint32_t used_us = pass_end ? _cpu_charge.pass_end(cpu_us, reset, wall_us) :
                                        _cpu_charge.pass_start(cpu_us, reset, wall_us);
    _host_cpu_us += used_us;
    _duty_cap.charge(used_us);
    if (pass_end) {
        _pass_cpu_max_us = MAX(_pass_cpu_max_us, used_us);
    }
}

static_assert(RustM4::ServeDutyCap::MAX_PAUSE_US <= RustM4::CALL_TIMEOUT_MS * 1000U * 3U / 5U,
              "a duty-cap pause leaves the M4's next call time to be served");

// listening: the ring was empty with the asleep flag set, so the M4 rings for its next frame
void AP_RustM4::pause_for_duty_cap(uint32_t pause_us, bool listening)
{
    const uint32_t until_us = AP_HAL::micros() + pause_us;
    if (!listening) {
        _channel->wake();   // frames are pending: no doorbell is needed while we pause
    }
    while (true) {
        const uint32_t now_us = AP_HAL::micros();
        if (int32_t(until_us - now_us) <= 0) {
            break;
        }
        const uint32_t remaining_us = until_us - now_us;
        if (!listening) {
            hal.scheduler->delay_microseconds(uint16_t(MIN(remaining_us, uint32_t(UINT16_MAX))));
        } else if (_doorbell.wait(remaining_us)) {
            listening = false;   // heard: the frame is served when the pause ends, no more doorbells
            _channel->wake();
        }
    }
}

// serve a bounded batch of M4 messages
AP_RustM4::ServeResult AP_RustM4::serve()
{
    if (_state != State::Running) {
        return ServeResult::Stuck;   // frames from before a (re)start are dropped by the re-handshake
    }
    const RustM4::PassEnd end = RustM4::serve_pass(
        [this](uint16_t &len) { return _channel->receive(_rx_frame, sizeof(_rx_frame), &len); },
        [this](uint16_t len) { handle_message(_rx_frame, len); },
        [this]() { return AP_HAL::micros() - _woke_us; });
    switch (end) {
    case RustM4::PassEnd::Idle:
        return ServeResult::Idle;
    case RustM4::PassEnd::ReceiveError:
        // -EBADMSG: corrupt or desynchronised ring; -EMSGSIZE: an oversized frame stays queued
        _protocol_errors++;
        return ServeResult::Stuck;
    case RustM4::PassEnd::Busy:
        break;
    }
    return ServeResult::Busy;
}

void AP_RustM4::handle_message(const uint8_t *buf, uint16_t len)
{
    RustM4::MsgHeader header;
    if (!RustM4::parse_header(buf, len, header)) {
        _protocol_errors++;   // an unknown frame type or a malformed frame
        return;
    }
    _requests++;
    switch (MsgType(header.type)) {
    case MsgType::Stats:
        log_rpc_stats(buf);
        return;
    case MsgType::BindingCall:
    case MsgType::SystemCall:
        return;   // not served yet: a SystemCall Noop notify is dropped unserved
    case MsgType::FaultInject:
    case MsgType::Ready:
    case MsgType::BindingReply:
    case MsgType::SystemReply:
    case MsgType::Control:
        _protocol_errors++;   // M7 -> M4 only
        return;
    }
}

void AP_RustM4::send_ready(uint32_t now_ms)
{
    RustM4::ReadyMsg ready;
    RustM4::fill_header(ready.header, MsgType::Ready, sizeof(ready), 0);
    _ready_sent_ms = now_ms;
    if (_channel->send(&ready, sizeof(ready)) != 0) {
        _dropped++;
    }
}

// one row per method; SemMax carries the largest M7-side time of one call
void AP_RustM4::log_rpc_stats(const uint8_t *buf)
{
    RustM4::StatsMsg stats;
    memcpy(&stats, buf, sizeof(stats));
    _m4_calls_lost += stats.lost;
    _rpc_summary.count += stats.count;
    _rpc_summary.p99_us = MAX(_rpc_summary.p99_us, stats.p99_us);
    _rpc_summary.max_us = MAX(_rpc_summary.max_us, stats.max_us);
    if (stats.count > _rpc_summary.busiest_count) {
        _rpc_summary.busiest_count = stats.count;
        _rpc_summary.p50_us = stats.p50_us;
    }
#if HAL_LOGGING_ENABLED
    // the M4 names its rows; the text is untrusted: terminated and printable before logging
    char name[sizeof(stats.name)];
    memcpy(name, stats.name, sizeof(name));
    name[sizeof(name) - 1] = 0;
    for (char *c = name; *c != 0; c++) {
        if (*c < 0x20 || *c > 0x7e) {
            *c = '?';
        }
    }
    // @LoggerMessage: SRPC
    // @Description: Rust M4 binding call round trips of one method, once per second (M4 builds)
    // @Field: TimeUS: Time since system startup
    // @Field: Id: method id
    // @Field: Name: method name, as the M4 reports it
    // @Field: N: round trips completed in the last second
    // @Field: P50: median round trip (log2 bucket bound)
    // @Field: P99: 99th percentile round trip (log2 bucket bound)
    // @Field: Max: longest round trip
    // @Field: SemMax: longest M7-side time of one call
    // @Field: Tot: total round-trip time
    AP::logger().WriteStreaming("SRPC", "TimeUS,Id,Name,N,P50,P99,Max,SemMax,Tot",
                                "s#--sssss", "F---FFFFF", "QHZIIIIIQ",
                                AP_HAL::micros64(), stats.method, name,
                                stats.count, stats.p50_us, stats.p99_us, stats.max_us, stats.m7_max_us,
                                uint64_t(stats.total_us));
#endif
}

// @LoggerMessage: M4ST
// @Description: Rust M4 supervisor, once per second (M4 builds)
// @Field: TimeUS: Time since system startup
// @Field: St: supervisor state (0 held, 1 starting, 2 running, 3 restarting, 4 off)
// @Field: HB: M4 heartbeat counter
// @Field: Ses: M4 peer header session
// @Field: Rst: restarts used
// @Field: Flt: M4 fault count
// @Field: FPC: PC of the last M4 fault
// @Field: Req: M4 messages served in the last second
// @Field: PErr: protocol errors in the last second
// @Field: Drop: frames dropped because the M4's ring was full
// @Field: Lost: M4 calls without a reply since the previous row
// @Field: Cap: pauses the host's duty cap (10 % of the M7's CPU) forced in the last second
// @Field: CPU: host thread CPU time in the last second, per mille of the M7 (what the duty cap charges)
// @Field: Lng: host passes over 50 ms of wall time in the last second (blocked in a binding, or preempted)
void AP_RustM4::log_supervisor(uint32_t now_ms, uint32_t elapsed_ms)
{
#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("M4ST", "TimeUS,St,HB,Ses,Rst,Flt,FPC,Req,PErr,Drop,Lost,Cap,CPU,Lng",
                                "s-------------", "F-------------", "QBIIBIIIHHIHHH",
                                AP_HAL::micros64(), uint8_t(_state), _m4_heartbeat, _m4_session,
                                _restarts_used, _m4_fault_count, _m4_fault_pc, _requests,
                                _protocol_errors, _dropped, _m4_calls_lost, _duty_cap_hits,
                                uint16_t(MIN(_host_cpu_us / MAX(elapsed_ms, 1U), uint32_t(UINT16_MAX))),
                                _long_passes);
    // @LoggerMessage: M4HP
    // @Description: Rust M4 host thread passes, once per second (M4 builds)
    // @Field: TimeUS: Time since system startup
    // @Field: PassMax: the largest CPU time of one pass in the last second, in microseconds (what the duty cap charges)
    // @Field: WallBusy: host thread wall time from wake to the end of each pass in the last second, per mille (thread statistics do not feed it; preemption counts, the switch around a pass does not)
    // @Field: StatsReset: thread statistics resets seen in the last second (an interval with one is charged the CPU since the reset plus 200 us, at most its wall time)
    AP::logger().WriteStreaming("M4HP", "TimeUS,PassMax,WallBusy,StatsReset", "s---", "F---", "QIHH",
                                AP_HAL::micros64(), _pass_cpu_max_us,
                                uint16_t(MIN(_host_busy_us / MAX(elapsed_ms, 1U), uint32_t(UINT16_MAX))),
                                _cpu_charge.take_resets());
#endif
    _m4_calls_lost = 0;
    if ((uint8_t(_options.get()) & OPTION_RPC_SUMMARY) == 0 || _state != State::Running ||
        now_ms - _last_summary_ms < 10000) {
        return;
    }
    const uint32_t window_ms = _last_summary_ms != 0 ? now_ms - _last_summary_ms : 10000;
    _last_summary_ms = now_ms;
    const uint32_t per_second = uint32_t(uint64_t(_rpc_summary.count) * 1000 / MAX(window_ms, 1U));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rust RPC %u/s p50<%u p99<%u max %uus",
                  unsigned(per_second), unsigned(_rpc_summary.p50_us),
                  unsigned(_rpc_summary.p99_us), unsigned(_rpc_summary.max_us));
    memset(&_rpc_summary, 0, sizeof(_rpc_summary));
    // busy microseconds per millisecond = per mille of one core
    const uint32_t host_permille = _host_busy_us / MAX(elapsed_ms, 1U);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rust M7 host %u.%u%% CPU",
                  unsigned(host_permille / 10), unsigned(host_permille % 10));
}

void AP_RustM4::supervise(uint32_t now_ms)
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

/*
  An M4 fault record as a short name for the one-line restart text ("MemManage", in SITL
  "signal 11") and its detail ("MemManage PC=0x0810A3C4", RustM4::fault_reason_text; in SITL the
  signal and address)
 */
static void fault_reason(const PeerHeader::View &view, char *name, size_t name_len, char *detail, size_t detail_len)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    hal.util->snprintf(name, name_len, "signal %u", unsigned(view.faultCfsr));
    hal.util->snprintf(detail, detail_len, "signal %u addr=0x%08X", unsigned(view.faultCfsr), unsigned(view.faultPc));
#else
    hal.util->snprintf(name, name_len, "%s", RustM4::fault_name(view.faultCfsr, view.faultHfsr));
    // the module stack is known once modules load on the M4
    RustM4::fault_reason_text(detail, detail_len, view.faultCfsr, view.faultHfsr, view.faultPc, view.faultLr, 0);
#endif
}

void AP_RustM4::check_started(uint32_t now_ms)
{
    PeerHeader::View view {};
    const PeerHeader::Status status = PeerHeader::readPeer(_shared, PeerHeader::Core::M7,
                                                           RustM4::PROTOCOL_VERSION,
                                                           RustM4::CONTRACT_HASH, &view);
    if (status == PeerHeader::Status::Mismatch) {
        go_off("image mismatch");
        return;
    }
    if (status == PeerHeader::Status::Compatible && view.restarts == _restart_grants &&
        (_state == State::Starting || view.session != _m4_session)) {
        _m4_session = view.session;
        // a fault restart has been charged: a fault the M4 recorded on its way down is the same
        // incident. A fault the new session recorded after its publish() and before this read
        // shows here too; check_running() reports it if the session never beats. After a requested
        // restart the count stays: a fault recorded around the request is still a fault, which
        // check_running() charges to RUST_M4_RST_MAX
        _start_fault.session_started(_state == State::Restarting && !_restart_requested,
                                     view.faultCount, _m4_fault_count);
        if (_state == State::Starting || !_restart_requested) {
            _m4_fault_count = view.faultCount;
        }
        _m4_heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
        _m4_heartbeat_ms = now_ms;
        // re-handshake: drop whatever either side queued before this start
        _channel->resetAfterRestart();
        _session_calls = 0;
        _module_count = 0;
        memset(&_rpc_summary, 0, sizeof(_rpc_summary));
        _restart_requested = false;
        enter(State::Running, now_ms);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rust: started on M4 (%u module%s, M4 RAM %uKB)", unsigned(_module_count),
                      _module_count == 1 ? "" : "s", unsigned(AP_RUST_M4_MODULE_AREA_SIZE / 1024U));
        // the M4 forwards module calls only after this
        send_ready(now_ms);
        return;
    }
    const uint32_t timeout_ms = _state == State::Starting ? RustM4::START_TIMEOUT_MS
                                                          : PLATFORM_RESTART_TIMEOUT_MS;
    if (platform_timeout_ms() - _state_start_timeout_ms > timeout_ms) {
        go_off("not responding");
    }
}

void AP_RustM4::check_running(uint32_t now_ms)
{
    if (platform_m4_exited()) {
        failure("process exit");
        return;
    }
    PeerHeader::View view {};
    const PeerHeader::Status status = PeerHeader::readPeer(_shared, PeerHeader::Core::M7,
                                                           RustM4::PROTOCOL_VERSION,
                                                           RustM4::CONTRACT_HASH, &view);
    if (status == PeerHeader::Status::Mismatch) {
        go_off("image mismatch");
        return;
    }
    if (status == PeerHeader::Status::Compatible) {
        if (view.faultCount != _m4_fault_count) {
            _m4_fault_count = view.faultCount;
            _m4_fault_pc = view.faultPc;
            char name[16];
            char detail[80];
            fault_reason(view, name, sizeof(name), detail, sizeof(detail));
            failure(name, detail);
            return;
        }
        if (view.session != _m4_session) {
            failure("self restart");
            return;
        }
    }
    // Busy: the M4 is rewriting its header (or died doing so); the heartbeat decides
    const uint32_t heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
    if (heartbeat != _m4_heartbeat) {
        _m4_heartbeat = heartbeat;
        _m4_heartbeat_ms = now_ms;
        _start_fault.beat();
    } else if (now_ms - _m4_heartbeat_ms > RustM4::HEARTBEAT_TIMEOUT_MS) {
        if (!report_start_fault()) {
            failure("no heartbeat");
        }
        return;
    }
    if (_protocol_errors >= PROTOCOL_ERROR_LIMIT) {
        failure("bad frames");
        return;
    }
    if (_session_calls == 0 && now_ms - _ready_sent_ms >= READY_REPEAT_MS) {
        send_ready(now_ms);
    }
}

// a fault restart's session that faulted after its publish() and before its first beat: report and
// charge that fault (name and PC) when its heartbeat times out. False when no such fault is pending
bool AP_RustM4::report_start_fault()
{
    PeerHeader::View view {};
    const bool readable = PeerHeader::readPeer(_shared, PeerHeader::Core::M7, RustM4::PROTOCOL_VERSION,
                                               RustM4::CONTRACT_HASH, &view) == PeerHeader::Status::Compatible;
    if (!_start_fault.report_on_timeout(readable)) {
        return false;
    }
    _m4_fault_pc = view.faultPc;
    char name[16];
    char detail[80];
    fault_reason(view, name, sizeof(name), detail, sizeof(detail));
    failure(name, detail);
    return true;
}

uint8_t AP_RustM4::restart_limit() const
{
    return uint8_t(constrain_int16(_restart_max.get(), 0, 10));
}

/*
  reason: short (at most 12 characters), so the restart text stays within one 50-character
  STATUSTEXT: "Rust: M4 fault (process exit), restarting (10/10)". detail: an M4 fault record's
  text, sent first on its own line
 */
void AP_RustM4::failure(const char *reason, const char *detail)
{
    if (detail != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Rust: M4 %s", detail);
    }
    const bool held_requests = _start_fault.take_held();
    if (_restarts_used >= restart_limit()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Rust: M4 fault (%s)", reason);
        go_off("fault limit");
        return;
    }
    _restarts_used++;
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Rust: M4 fault (%s), restarting (%u/%u)", reason,
                  unsigned(_restarts_used), unsigned(restart_limit()));
    _restart_requested = false;   // a fault during a requested restart is a fault
    grant_restart();
    // a restart asked while a start fault was pending is served by this fault restart
    if (held_requests) {
        bool restart;
        IGNORE_RETURN(_run_requests.take(restart));
    }
}

// a restart for the M4, a fault's or a request's: the grant count is its own, so requested restarts
// spend none of RUST_M4_RST_MAX (_restarts_used)
void AP_RustM4::grant_restart()
{
    _restart_grants++;
    // the new session starts without the old one's protocol errors
    _protocol_errors = 0;
    // the M4 acts on a restart doorbell only once the grant has changed: store it first
    RustM4::supervisor_block(_shared)->restart_grant.store(_restart_grants);
    platform_request_restart();
    enter(State::Restarting, AP_HAL::millis());
}

// RUST_RESTART changes (update()), taken while a session runs: a request made while the M4 starts
// or restarts waits in _run_requests, and an M4 that is off is never restarted
void AP_RustM4::run_control(uint32_t now_ms)
{
    // a fault at the session's start is pending: until the heartbeat tells whose it is (at most
    // HEARTBEAT_TIMEOUT_MS), a request waits, so it can never charge an absorbed fault again
    if (_start_fault.holds_requests()) {
        return;
    }
    if (take_run_request(now_ms - _state_start_ms) != RustM4::RunAction::None) {
        request_restart();
    }
}

// take a restart request and say what it does; running_ms: the session's age
RustM4::RunAction AP_RustM4::take_run_request(uint32_t running_ms)
{
    bool restart = false;
    const bool asked = _run_requests.take(restart);
    // RUST_ENABLE needs a reboot: a session that runs is enabled
    return _run_control.update(true, asked, restart, running_ms);
}

// a restart the M7 asks for: the session ends as after a fault, and the new one loads the modules
void AP_RustM4::request_restart()
{
    _restart_requested = true;
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
    // RUST_M4_INJECT 12 marks the next grant for a fault restart only
    RustM4::supervisor_block(_shared)->fault_after_publish_grant.store(0);
#endif
    grant_restart();
}

void AP_RustM4::go_off(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Rust: M4 %s, off until reboot", reason);
    if (_shared != nullptr) {
        RustM4::supervisor_block(_shared)->run_permit.store(0);
    }
    // after run_permit: stopping rings the restart doorbell, which the M4 reads as park
    platform_stop();
    enter(State::Off, AP_HAL::millis());
}

void AP_RustM4::enter(State state, uint32_t now_ms)
{
    _state = state;
    _state_start_ms = now_ms;
    _state_start_timeout_ms = platform_timeout_ms();
}

bool AP_RustM4::arming_checks(size_t buflen, char *buffer) const
{
    // Held: RUST_ENABLE was 0 at boot, as the in-process runtime passes with RUST_ENABLE 0
    if (_state == State::Held || _prearm_disabled) {
        return true;
    }
    if (_state == State::Off) {
        hal.util->snprintf(buffer, buflen, "Rust: M4 not running");
        return false;
    }
    if (_state != State::Running) {
        hal.util->snprintf(buffer, buflen, "Rust: M4 modules not started");
        return false;
    }
    return true;
}

#if AP_RUST_M4_FAULT_INJECTION_ENABLED
void AP_RustM4::send_fault_inject()
{
    const int8_t kind = _inject.get();
    if (kind == _inject_seen) {
        return;
    }
    _inject_seen = kind;
    if (kind <= 0 || _state != State::Running) {
        return;
    }
    uint32_t sent;
    if (kind == RustM4::INJECT_FAULT_AFTER_PUBLISH) {
        // the fault restart this HardFault causes is grant _restart_grants + 1
        RustM4::supervisor_block(_shared)->fault_after_publish_grant.store(_restart_grants + 1);
        sent = uint32_t(RustM4::FaultKind::HardFault);
    } else if (kind == RustM4::INJECT_MODULE_SELFTEST ||
               (kind <= RustM4::FAULT_KIND_MAX && kind != RustM4::INJECT_REFUSE_BINDINGS &&
                !RustM4::module_fault_kind(kind))) {
        // the service kinds and the thread-path selftest
        sent = uint32_t(kind);
    } else {
        // the module kinds: the test module reads RUST_M4_INJECT itself
        return;
    }
    RustM4::FaultInjectMsg inject;
    RustM4::fill_header(inject.header, MsgType::FaultInject, sizeof(inject), 0);
    inject.kind = sent;
    if (_channel->send(&inject, sizeof(inject)) != 0) {
        _dropped++;
        // no HardFault, so no fault restart for the mark
        RustM4::supervisor_block(_shared)->fault_after_publish_grant.store(0);
    }
}
#endif

#endif  // AP_RUST_M4_ENABLED
