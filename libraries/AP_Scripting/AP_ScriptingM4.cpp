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
#include "AP_ScriptingM4_Remote.h"
#include "AP_ScriptingM4_Rpc.h"
#include "AP_Scripting.h"

static_assert(HAL_WITH_EKF_DOUBLE && HAL_WITH_POSTYPE_DOUBLE,
              "the M4 computes Location math in double (m4/shim/AP_HAL/AP_HAL_Boards.h)");

extern const AP_HAL::HAL& hal;

namespace PeerHeader = CubePilotFW::PeerHeader;
using ScriptingM4::MsgType;

// one wake serves at most this many requests or this long (spec §7)
static const uint8_t MAX_REQUESTS_PER_WAKE = 32;
static const uint32_t MAX_SERVE_US = 200;
// after a pass that hit either limit with frames left: a flooding M4 gets one pass per pause
static const uint32_t SERVE_BACKOFF_US = 500;
// a pass this long in wall time is counted (M4ST.Long): blocked in a binding, or preempted
static const uint32_t LONG_PASS_US = 50000;
// protocol errors within one second that count as a failure
static const uint16_t PROTOCOL_ERROR_LIMIT = 3;
// Ready is repeated this often until the first call of a session
static const uint32_t READY_REPEAT_MS = 1000;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// a SITL restart waits on host process scheduling (kill, execv, dynamic loading), not on M4 boot time
static const uint32_t PLATFORM_RESTART_TIMEOUT_MS = ScriptingM4::START_TIMEOUT_MS;
#else
static const uint32_t PLATFORM_RESTART_TIMEOUT_MS = ScriptingM4::RESTART_TIMEOUT_MS;
#endif
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
// SCR_M4_INJECT value handled on the M7, in a gap of the M4's FaultKind values
static const int8_t INJECT_REFUSE_BINDINGS = 10;
static_assert(INJECT_REFUSE_BINDINGS > uint8_t(ScriptingM4::FaultKind::LuaRccWrite) &&
              INJECT_REFUSE_BINDINGS < uint8_t(ScriptingM4::FaultKind::HandlerFault), "not a FaultKind");
// SCR_M4_INJECT value handled on the M7: a HardFault now, and another right after the restarted
// M4 publishes its header, before the M7 reads it (an early fault)
static const int8_t INJECT_FAULT_AFTER_PUBLISH = 12;
static_assert(INJECT_FAULT_AFTER_PUBLISH > ScriptingM4::FAULT_KIND_MAX, "not sent to the M4");
#endif

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
    // @Values: 0:None,1:HardFault,2:MemManage,3:Stall,4:Lockup,5:Ring corruption,6:Request flood,7:Lua AIRCR reset,8:Lua stack overflow,9:Lua RCC write,10:Bindings image refused,11:Fault in an M4 interrupt handler,12:Fault again after the restart
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
    // the slow worker first: the host hands it every frame the fast lane does not serve. It runs
    // the heavy bindings (files, @SYS, logs, parameter names, mode changes) below io 58,
    // storage/log_io 59, UART/net 60 and the io - 1 threads (OA path planner, mount POI), with
    // the stack the host ran them with; its CPU is charged to the duty cap too
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ScriptingM4::worker_thread, void),
                                      "ScriptingM4W", 8192, AP_HAL::Scheduler::PRIORITY_IO,
                                      ScriptingM4::WORKER_PRIORITY_OFFSET)) {
        go_off("no worker thread");
        return;
    }
    // binding calls run vehicle code (set_target_location) on this thread, which the in-process
    // scripting thread ran with 17 KB. Above the I/O-class threads (io 58, storage/log_io 59,
    // UART/net 60) so a call is not queued behind logging; below every vehicle-timing thread (I2C
    // 176 and up). The duty cap bounds the CPU share a flooding M4 can take from those threads
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ScriptingM4::host_thread, void),
                                      "ScriptingM4", 8192, AP_HAL::Scheduler::PRIORITY_UART,
                                      ScriptingM4::HOST_PRIORITY_OFFSET)) {
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
                _slow_requests = 0;
                _slow_max_us = 0;
                _slow_cpu_max_us = 0;
            }
            hal.scheduler->delay(100);
            continue;
        }
        const uint32_t cap_wait_us = _duty_cap.wait_us(AP_HAL::micros());
        const bool ring_empty = _channel->prepareSleep();   // sets the asleep flag: the M4 rings from here on
        switch (ScriptingM4::host_wait_after(cap_wait_us, last == ServeResult::Busy, last == ServeResult::Stuck,
                                             ring_empty)) {
        case ScriptingM4::HostWait::DutyCap:
            // the host's share of the M7 ran out: serve nothing until credit returns, however many
            // frames are pending. An empty ring keeps listening, so its next doorbell is not missed
            _duty_cap_hits++;
            pause_for_duty_cap(MIN(cap_wait_us, platform_wait_us()), ring_empty);
            // frames pending after the pause are served at once (the cap still comes first)
            last = ServeResult::Idle;
            break;
        case ScriptingM4::HostWait::Backoff:
            // the budget ran out with frames left: pause so a flooding M4 costs bounded M7 time
            // (spec §7). No doorbell is needed while we pause
            _channel->wake();
            hal.scheduler->delay_microseconds(SERVE_BACKOFF_US);
            break;
        case ScriptingM4::HostWait::Doorbell:
        case ScriptingM4::HostWait::Timeout:
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
        charge_cpu(false);   // ChibiOS: the wait, the wake and the switch before this pass
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
        // the M4 asks for the image at every start: a value set before a start acts on it
        _server.set_refuse_bindings(_inject.get() == INJECT_REFUSE_BINDINGS);
#endif
        // the only call of serve(): a pause shorter than the cap's wait (the supervisor still
        // runs every platform_wait_us()) serves nothing
        if (_duty_cap.wait_us(_woke_us) == 0) {
            last = serve();
        }
        const uint32_t now_ms = AP_HAL::millis();
        PeerHeader::beat(_shared, PeerHeader::Core::M7);
        supervise(now_ms);
        if (_state == State::Running) {
            _server.update(now_ms);
            // after this pass's calls: a stop or restart never makes the host thread wait
            run_control(now_ms);
        }
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
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
            _slow_requests = 0;
            _slow_max_us = 0;
            _slow_cpu_max_us = 0;
        }
    }
}

// charge the duty cap the host's CPU since its previous reading (HostCpuCharge); at a pass end
// that is the pass, the largest of which M4HP logs
void AP_ScriptingM4::charge_cpu(bool pass_end)
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

// the slow worker: one frame at a time, handed over by the host, which waits until it is served.
// The CPU the frame used goes to the host, which charges it to the duty cap
void AP_ScriptingM4::worker_thread()
{
    while (true) {
        IGNORE_RETURN(_worker_request.wait_blocking());
        uint64_t cpu_us;
        bool reset = platform_thread_cpu_us(cpu_us);
        IGNORE_RETURN(_worker_cpu_charge.pass_start(cpu_us, reset, AP_HAL::micros()));
        handle_message(_rx_frame, _worker_len);
        reset = platform_thread_cpu_us(cpu_us);
        _worker_used_us = _worker_cpu_charge.pass_end(cpu_us, reset, AP_HAL::micros());
        _worker_done.signal();
    }
}

static_assert(ScriptingM4::ServeDutyCap::MAX_PAUSE_US <= ScriptingM4::CALL_TIMEOUT_MS * 1000U * 3U / 5U,
              "a duty-cap pause leaves the M4's next call time to be served");
static_assert(ScriptingM4Remote::METHOD_FAST_LANE == ScriptingM4::METHOD_FLAG_FAST_LANE, "one fast-lane flag");

bool AP_ScriptingM4::on_fast_lane(const uint8_t *buf, uint16_t len) const
{
    return ScriptingM4::frame_on_fast_lane(buf, len, ScriptingM4Remote::method_flags);
}

void AP_ScriptingM4::serve_on_worker(uint16_t len)
{
    const uint32_t start_us = AP_HAL::micros();
    _worker_len = len;
    _worker_request.signal();
    IGNORE_RETURN(_worker_done.wait_blocking());
    _slow_requests++;
    _slow_max_us = MAX(_slow_max_us, AP_HAL::micros() - start_us);
    // the worker's CPU counts against the same share as the host's: a script looping a heavy
    // frame (fs:crc32 of a large file) is paused like a flooding one
    _host_cpu_us += _worker_used_us;
    _duty_cap.charge(_worker_used_us);
    _slow_cpu_max_us = MAX(_slow_cpu_max_us, _worker_used_us);
}

// listening: the ring was empty with the asleep flag set, so the M4 rings for its next frame
void AP_ScriptingM4::pause_for_duty_cap(uint32_t pause_us, bool listening)
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
AP_ScriptingM4::ServeResult AP_ScriptingM4::serve()
{
    if (_state != State::Running) {
        return ServeResult::Stuck;   // frames from before a (re)start are dropped by the re-handshake
    }
    // an empty ring is Idle whatever the time; Busy means a limit ran out with a frame just served
    for (uint8_t i = 0; i < MAX_REQUESTS_PER_WAKE; i++) {
        uint16_t len = 0;
        const int ret = _channel->receive(_rx_frame, sizeof(_rx_frame), &len);
        if (ret == -EAGAIN) {
            return ServeResult::Idle;
        }
        if (ret != 0) {
            // -EBADMSG: corrupt or desynchronised ring; -EMSGSIZE: an oversized frame stays queued
            _protocol_errors++;
            return ServeResult::Stuck;
        }
        if (on_fast_lane(_rx_frame, len)) {
            handle_message(_rx_frame, len);
        } else {
            serve_on_worker(len);
        }
        if (AP_HAL::micros() - _woke_us > MAX_SERVE_US) {
            return ServeResult::Busy;
        }
    }
    return ServeResult::Busy;
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
    switch (MsgType(header.type)) {
    case MsgType::Call:
    case MsgType::Notify:
        handle_call(buf, len, header, start_us);
        return;
    case MsgType::Stats:
        log_rpc_stats(buf);
        return;
    case MsgType::Reply:
    case MsgType::FaultInject:
    case MsgType::Ready:
        _protocol_errors++;   // M7 -> M4 only
        return;
    }
}

// a Lua call: decode, let the server check and run it, reply with the M7 timings
void AP_ScriptingM4::handle_call(const uint8_t *buf, uint16_t len, const ScriptingM4::MsgHeader &header,
                                 uint32_t start_us)
{
    using namespace ScriptingM4;
    CallHeader call;
    memcpy(&call, buf, sizeof(call));   // parse_header() checked len >= sizeof(CallHeader)
    _session_calls++;
    RpcValue args[MAX_VALUES];
    RpcWriter results;
    results.begin(_tx_frame, sizeof(_tx_frame), sizeof(ReplyHeader));
    uint8_t error_arg = 0;
    ReplyStatus status = ReplyStatus::Malformed;
    if (call.argc <= MAX_VALUES && decode_values(buf + sizeof(call), len - sizeof(call), call.argc, args)) {
        status = _server.handle(call.method, args, call.argc, results, error_arg);
    }
    if (status == ReplyStatus::Malformed || status == ReplyStatus::UnknownMethod) {
        // no M4 runtime of this contract sends such a frame
        _protocol_errors++;
        results.begin(_tx_frame, sizeof(_tx_frame), sizeof(ReplyHeader));
    }
    if (MsgType(header.type) == MsgType::Notify) {
        return;
    }
    if (!results.ok()) {
        static const char too_large[] = "M4: results too large";
        results.begin(_tx_frame, sizeof(_tx_frame), sizeof(ReplyHeader));
        results.string(too_large, sizeof(too_large) - 1);
        status = ReplyStatus::Error;
    }
    ReplyHeader reply {};
    fill_header(reply.header, MsgType::Reply, results.length(), header.seq);
    reply.status = uint8_t(status);
    reply.count = results.count();
    reply.error_arg = error_arg;
    reply.m7_wake_us = _wake_latency_us;
    reply.m7_exec_us = AP_HAL::micros() - start_us;
    memcpy(_tx_frame, &reply, sizeof(reply));
    if (_channel->send(_tx_frame, results.length()) != 0) {
        _dropped++;   // the M4 times the call out
    }
}

void AP_ScriptingM4::send_ready(uint32_t now_ms)
{
    ScriptingM4::ReadyMsg ready;
    ScriptingM4::fill_header(ready.header, MsgType::Ready, sizeof(ready), 0);
    _ready_sent_ms = now_ms;
    if (_channel->send(&ready, sizeof(ready)) != 0) {
        _dropped++;
    }
}

// SRPC as in AP_Scripting_BindingStats.cpp, one row per method; SemMax carries the largest
// M7-side time of one call
void AP_ScriptingM4::log_rpc_stats(const uint8_t *buf)
{
    ScriptingM4::StatsMsg stats;
    memcpy(&stats, buf, sizeof(stats));
    _m4_calls_lost += stats.lost;
    _rpc_summary.count += stats.count;
    _rpc_summary.p99_us = MAX(_rpc_summary.p99_us, stats.p99_us);
    _rpc_summary.max_us = MAX(_rpc_summary.max_us, stats.max_us);
    // the m4:other and m4:sysother rows are sums of methods: their p50 is no median
    if (stats.method != uint16_t(ScriptingM4::SystemMethod::Other) &&
        stats.method != uint16_t(ScriptingM4::SystemMethod::SystemOther) && stats.count > _rpc_summary.busiest_count) {
        _rpc_summary.busiest_count = stats.count;
        _rpc_summary.p50_us = stats.p50_us;
    }
    // the M4 names its rows; the text is untrusted: terminated and printable before logging
    char name[sizeof(stats.name)];
    memcpy(name, stats.name, sizeof(name));
    name[sizeof(name) - 1] = 0;
    for (char *c = name; *c != 0; c++) {
        if (*c < 0x20 || *c > 0x7e) {
            *c = '?';
        }
    }
#if HAL_LOGGING_ENABLED && AP_SCRIPTING_RPC_STATS_ENABLED
    AP::logger().WriteStreaming("SRPC", "TimeUS,Id,Name,N,P50,P99,Max,SemMax,Tot",
                                "s#--sssss", "F---FFFFF", "QHZIIIIIQ",
                                AP_HAL::micros64(), stats.method, name,
                                stats.count, stats.p50_us, stats.p99_us, stats.max_us, stats.m7_max_us,
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
// @Field: Lost: M4 calls without a reply since the previous row
// @Field: QHit: M4 calls refused by their quota since the previous row: binding calls, print, the M4's statustexts and script error texts, its file writes, write opens and removes (FileIo), MAVLink sends (MavTx), log rows (Log) and OSD calls (Osd, OsdFrame) (over quota, texts and void calls are dropped, boolean calls return false)
// @Field: Cap: pauses the host's duty cap (10 % of the M7's CPU) forced in the last second
// @Field: CPU: host thread CPU time in the last second, per mille of the M7 (what the duty cap charges)
// @Field: Lng: host passes over 50 ms of wall time in the last second (blocked in a binding, or preempted)
void AP_ScriptingM4::log_supervisor(uint32_t now_ms, uint32_t elapsed_ms)
{
#if HAL_LOGGING_ENABLED && AP_SCRIPTING_RPC_STATS_ENABLED
    AP::logger().WriteStreaming("M4ST", "TimeUS,St,HB,Ses,Rst,Flt,FPC,Req,PErr,Drop,Lost,QHit,Cap,CPU,Lng",
                                "s--------------", "F--------------", "QBIIBIIIHHIIHHH",
                                AP_HAL::micros64(), uint8_t(_state), _m4_heartbeat, _m4_session,
                                _restarts_used, _m4_fault_count, _m4_fault_pc, _requests,
                                _protocol_errors, _dropped, _m4_calls_lost, _server.take_quota_hits(),
                                _duty_cap_hits,
                                uint16_t(MIN(_host_cpu_us / MAX(elapsed_ms, 1U), uint32_t(UINT16_MAX))),
                                _long_passes);
#endif
#if HAL_LOGGING_ENABLED && AP_SCRIPTING_RPC_STATS_ENABLED
    // @LoggerMessage: M4HP
    // @Description: Scripting M4 host thread passes, once per second (M4 scripting builds)
    // @Field: TimeUS: Time since system startup
    // @Field: PassMax: the largest CPU time of one fast-lane pass in the last second, in microseconds (what the duty cap charges)
    // @Field: WallBusy: host thread wall time from wake to the end of each pass in the last second, per mille (thread statistics do not feed it; preemption and waits on the slow worker count, the switch around a pass does not)
    // @Field: StatsReset: thread statistics resets seen in the last second, host and slow worker (an interval with one is charged its wall time)
    // @Field: SlowReq: frames the slow worker served in the last second
    // @Field: SlowMax: the longest of them, wall time in microseconds
    // @Field: SlowCPU: the largest CPU time of one of them, in microseconds (charged to the duty cap with the host's)
    AP::logger().WriteStreaming("M4HP", "TimeUS,PassMax,WallBusy,StatsReset,SlowReq,SlowMax,SlowCPU", "s------",
                                "F------", "QIHHIII", AP_HAL::micros64(), _pass_cpu_max_us,
                                uint16_t(MIN(_host_busy_us / MAX(elapsed_ms, 1U), uint32_t(UINT16_MAX))),
                                uint16_t(_cpu_charge.take_resets() + _worker_cpu_charge.take_resets()), _slow_requests,
                                _slow_max_us, _slow_cpu_max_us);
#endif
    _m4_calls_lost = 0;
    if (!_summary_enabled || _state != State::Running || now_ms - _last_summary_ms < 10000) {
        return;
    }
    const uint32_t window_ms = _last_summary_ms != 0 ? now_ms - _last_summary_ms : 10000;
    _last_summary_ms = now_ms;
    const uint32_t per_second = uint32_t(uint64_t(_rpc_summary.count) * 1000 / MAX(window_ms, 1U));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting RPC %u/s p50<%u p99<%u max %uus",
                  unsigned(per_second), unsigned(_rpc_summary.p50_us),
                  unsigned(_rpc_summary.p99_us), unsigned(_rpc_summary.max_us));
    memset(&_rpc_summary, 0, sizeof(_rpc_summary));
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

// the GCS reason of the fault an M4 recorded: "HardFault PC=0x0810A3C4", in SITL the signal and address
static void fault_reason(const PeerHeader::View &view, char *reason, size_t reason_len)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    hal.util->snprintf(reason, reason_len, "signal %u addr=0x%08X",
                       unsigned(view.faultCfsr), unsigned(view.faultPc));
#else
    hal.util->snprintf(reason, reason_len, "%s PC=0x%08X",
                       ScriptingM4::fault_name(view.faultCfsr, view.faultHfsr), unsigned(view.faultPc));
#endif
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
    if (status == PeerHeader::Status::Compatible && view.restarts == _restart_grants &&
        (_state == State::Starting || view.session != _m4_session)) {
        _m4_session = view.session;
        // a fault restart has been charged: a fault the M4 recorded on its way down is the same
        // incident. A fault the new session recorded after its publish() and before this read
        // shows here too; check_running() reports it if the session never beats. After a requested
        // restart the count stays: a fault recorded around the request is still a fault, which
        // check_running() charges to SCR_M4_RST_MAX
        _start_fault.session_started(_state == State::Restarting && !_restart_requested,
                                     view.faultCount, _m4_fault_count);
        if (_state == State::Starting || !_restart_requested) {
            _m4_fault_count = view.faultCount;
        }
        _m4_heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
        _m4_heartbeat_ms = now_ms;
        // re-handshake: drop whatever either side queued before this start
        _channel->resetAfterRestart();
        _server.reset(_m4_session);
        _session_calls = 0;
        memset(&_rpc_summary, 0, sizeof(_rpc_summary));
        if (_state == State::Starting) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: M4 started");
        } else if (!_restart_requested) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: M4 restarted (%u/%u)",
                          unsigned(_restarts_used), unsigned(restart_limit()));
        }
        _restart_requested = false;
        enter(State::Running, now_ms);
        // the M4 forwards Lua calls only after this
        send_ready(now_ms);
        return;
    }
    const uint32_t timeout_ms = _state == State::Starting ? ScriptingM4::START_TIMEOUT_MS
                                                          : PLATFORM_RESTART_TIMEOUT_MS;
    if (platform_timeout_ms() - _state_start_timeout_ms > timeout_ms) {
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
            fault_reason(view, reason, sizeof(reason));
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
        _start_fault.beat();
    } else if (now_ms - _m4_heartbeat_ms > ScriptingM4::HEARTBEAT_TIMEOUT_MS) {
        if (!report_start_fault()) {
            failure("heartbeat lost");
        }
        return;
    }
    if (_protocol_errors >= PROTOCOL_ERROR_LIMIT) {
        failure("protocol errors");
        return;
    }
    if (_session_calls == 0 && now_ms - _ready_sent_ms >= READY_REPEAT_MS) {
        send_ready(now_ms);
    }
}

// a fault restart's session that faulted after its publish() and before its first beat: report and
// charge that fault (name and PC) when its heartbeat times out. False when no such fault is pending
bool AP_ScriptingM4::report_start_fault()
{
    PeerHeader::View view {};
    const bool readable = PeerHeader::readPeer(_shared, PeerHeader::Core::M7, ScriptingM4::PROTOCOL_VERSION,
                                               ScriptingM4::CONTRACT_HASH, &view) == PeerHeader::Status::Compatible;
    if (!_start_fault.report_on_timeout(readable)) {
        return false;
    }
    _m4_fault_pc = view.faultPc;
    char reason[40];
    fault_reason(view, reason, sizeof(reason));
    failure(reason);
    return true;
}

uint8_t AP_ScriptingM4::restart_limit() const
{
    return uint8_t(constrain_int16(_restart_max.get(), 0, 10));
}

void AP_ScriptingM4::failure(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: M4 %s", reason);
    const bool carry_request = _start_fault.take_held();
    if (_restarts_used >= restart_limit()) {
        go_off("fault limit");
        return;
    }
    _restarts_used++;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Scripting: M4 restarting (%u/%u)",
                  unsigned(_restarts_used), unsigned(restart_limit()));
    _restart_requested = false;   // a fault during a requested restart is a fault
    grant_restart();
    // requests held while a start fault was pending: this fault restart carries a stop asked meanwhile
    if (carry_request && take_run_request(0) == ScriptingM4::RunAction::Stop) {
        _server.set_stopped(true);
    }
}

// a restart for the M4, a fault's or a request's: the grant count is its own, so requested restarts
// spend none of SCR_M4_RST_MAX (_restarts_used)
void AP_ScriptingM4::grant_restart()
{
    _restart_grants++;
    // the new session starts without the old one's protocol errors
    _protocol_errors = 0;
    // the M4 acts on a restart doorbell only once the grant has changed: store it first
    ScriptingM4::supervisor_block(_shared)->restart_grant.store(_restart_grants);
    platform_request_restart();
    enter(State::Restarting, AP_HAL::millis());
}

// MAV_CMD_SCRIPTING, scripting:restart_all() and SCR_ENABLE, as AP_Scripting::thread() acts on
// them in-process (RunControl), with its texts. Running sessions only: a request made while the
// M4 starts or restarts waits in AP_Scripting's flags, and an M4 that is off is never restarted
void AP_ScriptingM4::run_control(uint32_t now_ms)
{
    // a fault at the session's start is pending: until the heartbeat tells whose it is (at most
    // HEARTBEAT_TIMEOUT_MS), a request waits, so it can never charge an absorbed fault again
    if (_start_fault.holds_requests()) {
        return;
    }
    const ScriptingM4::RunAction action = take_run_request(now_ms - _state_start_ms);
    if (action != ScriptingM4::RunAction::None) {
        request_restart(action == ScriptingM4::RunAction::Stop);
    }
}

// take a stop or restart request and say what it does; running_ms: the session's age
ScriptingM4::RunAction AP_ScriptingM4::take_run_request(uint32_t running_ms)
{
    AP_Scripting *scripting = AP_Scripting::get_singleton();
    bool restart = false;
    const bool stop = scripting->take_stop_request(restart);
    const bool was_stopped = _run_control.stopped();
    const ScriptingM4::RunAction action = _run_control.update(scripting->enabled(), stop, restart, running_ms);
    if (action == ScriptingM4::RunAction::None) {
        return action;
    }
    if (action == ScriptingM4::RunAction::Stop || !was_stopped) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: %s", "stopped");
    }
    if (action == ScriptingM4::RunAction::Restart) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: %s", "restarted");
    }
    return action;
}

// a restart the M7 asks for: the session ends as after a fault (end_session() at the new
// session's reset()), and the new one lists the scripts or, stopped, none
void AP_ScriptingM4::request_restart(bool stopped)
{
    _server.set_stopped(stopped);
    _restart_requested = true;
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    // SCR_M4_INJECT 12 marks the next grant for a fault restart only
    ScriptingM4::supervisor_block(_shared)->fault_after_publish_grant.store(0);
#endif
    grant_restart();
}

void AP_ScriptingM4::go_off(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Scripting: M4 %s, off until reboot", reason);
    if (_shared != nullptr) {
        ScriptingM4::supervisor_block(_shared)->run_permit.store(0);
    }
    // after run_permit: stopping rings the restart doorbell, which the M4 reads as park
    platform_stop();
    // an M4 that is off never closes its files or frees its objects and devices. A stale handle
    // or object entry cannot be reached afterwards: the M4 is stopped and the next start resets
    _server.end_session();
    enter(State::Off, AP_HAL::millis());
}

void AP_ScriptingM4::enter(State state, uint32_t now_ms)
{
    _state = state;
    _state_start_ms = now_ms;
    _state_start_timeout_ms = platform_timeout_ms();
}

bool AP_ScriptingM4::arming_checks(size_t buflen, char *buffer) const
{
    if (_state == State::Off || _state == State::Held) {
        hal.util->snprintf(buffer, buflen, "Scripting core not running");
        return false;
    }
    if (_state != State::Running) {
        // (re)starting: the server still holds the previous session
        hal.util->snprintf(buffer, buflen, "Scripting: M4 scripts not started");
        return false;
    }
    return _server.arming_checks(buflen, buffer);
}

#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
void AP_ScriptingM4::send_fault_inject()
{
    const int8_t kind = _inject.get();
    if (kind == _inject_seen) {
        return;
    }
    _inject_seen = kind;
    if (kind <= 0 || _state != State::Running) {
        return;
    }
    ScriptingM4::FaultKind sent;
    if (kind == INJECT_FAULT_AFTER_PUBLISH) {
        // the fault restart this HardFault causes is grant _restart_grants + 1
        ScriptingM4::supervisor_block(_shared)->fault_after_publish_grant.store(_restart_grants + 1);
        sent = ScriptingM4::FaultKind::HardFault;
    } else if (kind > ScriptingM4::FAULT_KIND_MAX || kind == INJECT_REFUSE_BINDINGS) {
        return;
    } else {
        sent = ScriptingM4::FaultKind(kind);
    }
    ScriptingM4::FaultInjectMsg inject;
    ScriptingM4::fill_header(inject.header, MsgType::FaultInject, sizeof(inject), 0);
    inject.kind = uint32_t(sent);
    if (_channel->send(&inject, sizeof(inject)) != 0) {
        _dropped++;
        // no HardFault, so no fault restart for the mark
        ScriptingM4::supervisor_block(_shared)->fault_after_publish_grant.store(0);
    }
}
#endif

#endif  // AP_SCRIPTING_M4_ENABLED
