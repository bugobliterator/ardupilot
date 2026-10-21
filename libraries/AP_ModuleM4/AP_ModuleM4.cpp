#include "AP_ModuleM4.h"

#if AP_MODULE_M4_ENABLED

#include <errno.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/CallLatencyLog.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include "AP_ModuleM4_CallStatus.h"
#include "AP_ModuleM4_Crc32.h"
#include "AP_ModuleM4_Layout.h"
#include "AP_ModuleM4_LoadLayout.h"

#if !AP_PARAM_CHANGE_OBSERVER_ENABLED
#error "AP_ModuleM4 needs AP_PARAM_CHANGE_OBSERVER_ENABLED"
#endif

static_assert(HAL_WITH_EKF_DOUBLE && HAL_WITH_POSTYPE_DOUBLE,
              "the M4 computes Location math in double (m4/shim/AP_HAL/AP_HAL_Boards.h)");

extern const AP_HAL::HAL& hal;

namespace PeerHeader = CubePilotFW::PeerHeader;
using ModuleM4::MsgType;
using AP_Module::CallLatencyLog;

static_assert(ModuleM4::PROBE_M4_SLEPT == AP_Module::CALL_M4_SLEPT && ModuleM4::PROBE_M4_POLL_WAKE == AP_Module::CALL_M4_POLL_WAKE &&
              ModuleM4::PROBE_M4_SENT_BY_STEP == AP_Module::CALL_M4_SENT_BY_STEP, "the M4's probe flags are the log's");
static const uint16_t PROBE_M4_FLAGS = ModuleM4::PROBE_M4_SLEPT | ModuleM4::PROBE_M4_POLL_WAKE | ModuleM4::PROBE_M4_SENT_BY_STEP;
static_assert(ModuleM4::PROBE_STAMP_PACKED == AP_Module::CALL_STAMP_M4_PACKED &&
              ModuleM4::PROBE_STAMP_POSTED == AP_Module::CALL_STAMP_M4_POSTED &&
              ModuleM4::PROBE_STAMP_FORWARD_RETURNED == AP_Module::CALL_STAMP_M4_FORWARD_RETURNED &&
              ModuleM4::PROBE_STAMP_REPLY_WAKE == AP_Module::CALL_STAMP_M4_REPLY_WAKE &&
              ModuleM4::PROBE_STAMP_SIGNALLED == AP_Module::CALL_STAMP_M4_SIGNALLED &&
              ModuleM4::PROBE_STAMP_RESUMED == AP_Module::CALL_STAMP_M4_RESUMED, "the M4's stamp bits are the trace's");

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
static const uint32_t PLATFORM_RESTART_TIMEOUT_MS = ModuleM4::START_TIMEOUT_MS;
#else
static const uint32_t PLATFORM_RESTART_TIMEOUT_MS = ModuleM4::RESTART_TIMEOUT_MS;
#endif
// MOD_DEBUG_OPTS bit that disables the module pre-arm checks (AP_Module::Runtime)
static const uint8_t DEBUG_OPTION_NO_PREARM = 1U << 4;
// the PR's MOD_LD_CSUM / MOD_RUN_CSUM mask
static const uint32_t CHECKSUM_MASK = 0x007FFFFF;

AP_ModuleM4 *AP_ModuleM4::_singleton;

const AP_Param::GroupInfo AP_ModuleM4::var_info[] = {
    // @Param: RST_MAX
    // @DisplayName: M4 module restart limit
    // @Description: M4 restarts after faults before native modules stay off until reboot
    // @Range: 0 10
    // @User: Advanced
    AP_GROUPINFO("RST_MAX", 1, AP_ModuleM4, _restart_max, 3),
#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
    // @Param: INJECT
    // @DisplayName: M4 module fault injection
    // @Description: Test builds only: performs this fault kind on the M4 once when set
    // @User: Advanced
    AP_GROUPINFO("INJECT", 2, AP_ModuleM4, _inject, 0),
#endif
    // @Param: OPTS
    // @DisplayName: M4 module options
    // @Description: Native modules on the Cortex-M4 options
    // @Bitmask: 0:GCS RPC summary every 10 s,1:GCS M4 CRC self-check values once per session,2:Call latency probe (M4LT log rows and @SYS/module_latency.txt, from the next M4 session)
    // @User: Advanced
    AP_GROUPINFO("OPTS", 3, AP_ModuleM4, _options, 0),
    AP_GROUPEND
};

AP_ModuleM4::AP_ModuleM4()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_ModuleM4::init(const AP_Module::Runtime::Config &config)
{
    if (_initialized) {
        return;
    }
    _initialized = true;
    _observed_restart = config.restart;
    _enabled = config.enable != 0;
    if (config.enable == 0) {
        return;   // Held: the M4 stays in reset (MOD_ENABLE needs a reboot)
    }
#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
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
    IGNORE_RETURN(AP_Param::set_change_observer(&AP_ModuleM4::parameter_changed));
    // the slow worker first: the host hands it every frame the fast lane does not serve. It runs
    // the heavy bindings (files, logs, parameters, mission, texts) below io 58, storage/log_io 59,
    // UART/net 60 and the io - 1 threads (OA path planner, mount POI); its CPU is charged to the
    // duty cap too
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ModuleM4::worker_thread, void),
                                      "ModuleM4W", 8192, ModuleM4::WORKER_PRIORITY_BASE,
                                      ModuleM4::WORKER_PRIORITY_OFFSET)) {
        go_off("no worker thread");
        return;
    }
    // binding calls run vehicle code on this thread. Above the I/O-class threads (io 58,
    // storage/log_io 59, UART/net 60) so a call is not queued behind logging; below every
    // vehicle-timing thread (I2C 176 and up). The duty cap bounds the CPU share a flooding M4
    // can take from those threads
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ModuleM4::host_thread, void),
                                      "ModuleM4", 8192, ModuleM4::HOST_PRIORITY_BASE,
                                      ModuleM4::HOST_PRIORITY_OFFSET)) {
        go_off("no host thread");
    }
}

void AP_ModuleM4::update(const AP_Module::Runtime::Config &config)
{
    if (!_initialized) {
        return;
    }
    // MOD_ENABLE 0 stops the modules (a restart into a session that loads none), 1 starts them
    // again; an M4 held at boot stays held until reboot
    _enabled = config.enable != 0;
    // MOD_RESTART is a counter: any change asks for a restart of the modules, which on the M4 is
    // a requested M4 restart (no fault, none of MOD_M4_RST_MAX spent)
    if (config.restart != _observed_restart) {
        _observed_restart = config.restart;
        _run_requests.restart();
    }
}

void AP_ModuleM4::doorbell_isr(uint8_t group)
{
    (void)group;
    if (_singleton != nullptr) {
        _singleton->_doorbell_us = AP_HAL::micros();
        if (_singleton->_probing) {
            _singleton->_doorbell_probe_us = CallLatencyLog::clock_us();
            _singleton->_doorbell_probed = true;
        }
        _singleton->_doorbell.signal_ISR();
    }
}

void AP_ModuleM4::host_thread()
{
    ServeResult last = ServeResult::Idle;
    bool woke_on_timeout = true;   // the first wake runs the housekeeping
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
                _slow_refused = 0;
                _m4_mirror_hits = 0;
                _m4_mirror_misses = 0;
                _m4_post_drops = 0;
                _mirror_frames_sent = 0;
                _mirror_entries_sent = 0;
                _mirror_failed_sends = 0;
                _post_errors = 0;
                _post_drops = 0;
                _posted_batched = 0;
            }
            hal.scheduler->delay(100);
            continue;
        }
        const uint32_t cap_wait_us = _duty_cap.wait_us(AP_HAL::micros());
        const bool ring_empty = _channel->prepareSleep();   // sets the asleep flag: the M4 rings from here on
        const ModuleM4::HostWait wait = ModuleM4::host_wait_after(cap_wait_us, last == ServeResult::Busy,
                                                              last == ServeResult::Stuck, ring_empty);
        woke_on_timeout = false;
        switch (wait) {
        case ModuleM4::HostWait::DutyCap:
            // the host's share of the M7 ran out: serve nothing until credit returns, however many
            // frames are pending. An empty ring keeps listening, so its next doorbell is not missed
            _duty_cap_hits++;
            pause_for_duty_cap(MIN(cap_wait_us, platform_wait_us()), ring_empty);
            // frames pending after the pause are served at once (the cap still comes first)
            last = ServeResult::Idle;
            break;
        case ModuleM4::HostWait::Backoff:
            // the budget ran out with frames left: pause so a flooding M4 costs bounded M7 time.
            // No doorbell is needed while we pause
            _channel->wake();
            hal.scheduler->delay_microseconds(SERVE_BACKOFF_US);
            break;
        case ModuleM4::HostWait::Doorbell:
        case ModuleM4::HostWait::Timeout:
            // a ring that cannot be served now (not running, or a frame stuck after
            // a protocol error) must not make this loop spin: wait for the timeout
            if (ring_empty || last == ServeResult::Stuck) {
                woke_on_timeout = !_doorbell.wait(platform_wait_us());
            }
            break;
        }
        // a doorbell wake: the frames between the pass's two CPU readings. The rest waits for a
        // timed-out wait or the next housekeeping period
        _channel->wake();
        _woke_us = AP_HAL::micros();
        if (_probing) {
            _pass_us = CallLatencyLog::clock_us();
            _pass_frames = 0;
            _pass_cap_paused = wait == ModuleM4::HostWait::DutyCap;
            // the doorbell interrupt that ended this wait, if any
            _pass_doorbell = _doorbell_probed;
            _pass_doorbell_us = _doorbell_probe_us;
            _doorbell_probed = false;
        }
        const uint32_t doorbell_us = _doorbell_us;
        _doorbell_us = 0;
        _wake_latency_us = doorbell_us != 0 ? _woke_us - doorbell_us : 0;
        // the only call of serve(): a pause shorter than the cap's wait (the supervisor still
        // runs every platform_wait_us()) serves nothing. ChibiOS: the wait, the wake and the
        // switch before this pass are charged after the decision
        if (ModuleM4::pass_after_wake(_duty_cap, _woke_us, [this]() { IGNORE_RETURN(charge_cpu(false)); })) {
            last = serve();
        }
        if (_state == State::Running && _server.mirror_dirty()) {
            mirror_on_worker();
        }
        if (ModuleM4::housekeeping_due(_woke_us, _housekeeping_us, ModuleM4::HOUSEKEEPING_PERIOD_US, woke_on_timeout)) {
            _housekeeping_us = _woke_us;
            housekeeping();
        }
        const uint32_t busy_us = charge_cpu(true) - _woke_us;
        _host_busy_us += busy_us;
        if (busy_us > LONG_PASS_US) {
            _long_passes++;
        }
    }
}

void AP_ModuleM4::housekeeping()
{
    _probing = (uint8_t(_options.get()) & OPTION_CALL_PROBE) != 0 && CallLatencyLog::get().start();
    if (_probing && !_input_memset_measured) {
        // the slow worker is idle between the host's passes
        _input_memset_measured = true;
        CallLatencyLog::get().set_m7_memset_cycles(_server.input_memset_cycles());
    }
    const uint32_t now_ms = AP_HAL::millis();
    PeerHeader::beat(_shared, PeerHeader::Core::M7);
    supervise(now_ms);
    if (_state == State::Running && now_ms - _mirror_backstop_ms >= 1000) {
        _mirror_backstop_ms = now_ms;
        _server.mark_all_live();   // raw set()s, whole-vector saves, failed frames
    }
    drain_module_texts();
    if (_state == State::Running) {
        // after this pass's frames: a restart never makes the host thread wait
        run_control(now_ms);
    }
#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
    send_fault_inject();
#endif
    if (_probing) {
        CallLatencyLog::get().flush(now_ms);
    }
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
        _slow_refused = 0;
        _m4_mirror_hits = 0;
        _m4_mirror_misses = 0;
        _m4_post_drops = 0;
        _mirror_frames_sent = 0;
        _mirror_entries_sent = 0;
        _mirror_failed_sends = 0;
        _post_errors = 0;
        _post_drops = 0;
        _posted_batched = 0;
    }
}

// charge the duty cap the host's CPU since its previous reading (HostCpuCharge); at a pass end
// that is the pass, the largest of which M4HP logs
uint32_t AP_ModuleM4::charge_cpu(bool pass_end)
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
    return wall_us;
}

// the slow worker: one frame at a time, handed over by the host, which waits until it is served.
// The CPU the frame used goes to the host, which charges it to the duty cap
void AP_ModuleM4::worker_thread()
{
    while (true) {
        IGNORE_RETURN(_worker_request.wait_blocking());
        uint64_t cpu_us;
        bool reset = platform_thread_cpu_us(cpu_us);
        IGNORE_RETURN(_worker_cpu_charge.pass_start(cpu_us, reset, AP_HAL::micros()));
        switch (_worker_job) {
        case WorkerJob::Discover:
            discover();
            break;
        case WorkerJob::Mirror:
            send_mirror_changes();
            break;
        case WorkerJob::Frame: {
            const bool posted = _server.frame_is_posted(_rx_frame, _worker_len);
            handle_message(_rx_frame, _worker_len);
            _worker_handback_len = 0;
            if (posted) {
                const uint32_t batch_start_us = AP_HAL::micros();
                _worker_handback_len = ModuleM4::serve_posted_batch(
                    _rx_frame,
                    [this](uint16_t &len) { return _channel->receive(_rx_frame, sizeof(_rx_frame), &len); },
                    [this](const uint8_t *frame, uint16_t len) {
                        return _server.frame_is_posted(frame, len) && !_duty_cap.refuses_slow_bindings();
                    },
                    [this](uint16_t len) {
                        _posted_batched++;
                        handle_message(_rx_frame, len);
                    },
                    [batch_start_us]() { return AP_HAL::micros() - batch_start_us; });
            }
            break;
        }
        }
        reset = platform_thread_cpu_us(cpu_us);
        _worker_used_us = _worker_cpu_charge.pass_end(cpu_us, reset, AP_HAL::micros());
        _worker_done.signal();
    }
}

void AP_ModuleM4::serve_on_worker(uint16_t len)
{
    const uint32_t start_us = AP_HAL::micros();
    _worker_job = WorkerJob::Frame;
    _worker_len = len;
    _worker_request.signal();
    IGNORE_RETURN(_worker_done.wait_blocking());
    _slow_requests++;
    _slow_max_us = MAX(_slow_max_us, AP_HAL::micros() - start_us);
    // the worker's CPU counts against the same share as the host's: a module looping a heavy
    // binding is paused like a flooding one
    _host_cpu_us += _worker_used_us;
    _duty_cap.charge(_worker_used_us);
    _slow_cpu_max_us = MAX(_slow_cpu_max_us, _worker_used_us);
}

// the directory reads at a session's start run on the slow worker, as the file calls do; the host
// waits for them before it sends Ready
void AP_ModuleM4::discover_on_worker()
{
    _worker_job = WorkerJob::Discover;
    _worker_request.signal();
    IGNORE_RETURN(_worker_done.wait_blocking());
    _host_cpu_us += _worker_used_us;
    _duty_cap.charge(_worker_used_us);
}

// the AP_Param change observer: any thread, lock-free (AP_ModuleM4_Server::mark_changed())
void AP_ModuleM4::parameter_changed(const AP_Param *param)
{
    AP_ModuleM4 *module_m4 = _singleton;
    if (module_m4 != nullptr) {
        module_m4->_server.mark_changed(param);
    }
}

void AP_ModuleM4::mirror_on_worker()
{
    _worker_job = WorkerJob::Mirror;
    _worker_request.signal();
    IGNORE_RETURN(_worker_done.wait_blocking());
    _host_cpu_us += _worker_used_us;
    _duty_cap.charge(_worker_used_us);
}

// ParameterChanged frames until no dirty slot has a change; a full ring leaves the rest dirty for
// the next pass, and the module's next reply asks the M4 to forward its gets until then
void AP_ModuleM4::send_mirror_changes()
{
    while (true) {
        const uint16_t len = _server.pack_mirror_changes(_mirror_frame, sizeof(_mirror_frame));
        if (len == 0) {
            return;
        }
        if (_channel->send(_mirror_frame, len) != 0) {
            _server.mirror_send_failed(_mirror_frame, len);
            _mirror_failed_sends++;
            return;
        }
        _mirror_frames_sent++;
        _mirror_entries_sent += (len - sizeof(ModuleM4::ParameterChangedHeader)) / sizeof(ModuleM4::ParameterValue);
    }
}

// the in-process runtime's discovery (AP_ModuleM4_Discovery.h); the milestone runs one module
void AP_ModuleM4::discover()
{
    _load_error[0] = 0;
    const AP_Module::Runtime *runtime = AP_Module::Runtime::get_singleton();
    const int8_t max_modules = runtime != nullptr ? runtime->config().max_modules : 0;
    ModuleM4::DiscoveryError error;
    uint8_t count = ModuleM4::discover_modules(ModuleM4::filesystem_lister(),
                                             uint8_t(constrain_int16(max_modules, 0, ModuleM4::MAX_MODULES)),
                                             _modules, ARRAY_SIZE(_modules), error);
    switch (error) {
    case ModuleM4::DiscoveryError::None:
        break;
    case ModuleM4::DiscoveryError::DirectoryUnavailable:
        load_error("module directory unavailable");
        break;
    case ModuleM4::DiscoveryError::TooMany:
    case ModuleM4::DiscoveryError::NameTooLong:
        load_error("directory read, name length or module count");
        break;
    }
    if (count > 1) {
        load_error("M4 runs 1 module");
        count = 0;
    }
    _module_count = count;
}

// a load failure of this session, in the PR's form ("Module: <reason>"), kept for the pre-arm check
void AP_ModuleM4::load_error(const char *reason)
{
    hal.util->snprintf(_load_error, sizeof(_load_error), "Module: %s", reason);
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s", _load_error);
}

/*
  In a pass the duty cap allows while its debt is beyond MAX_DEBT_US: a binding call that would go
  to the slow worker is not run but answered BUSY at once. False for anything else (system calls,
  which do bounded work per frame, and frames that do not parse), which the worker serves as usual
 */
bool AP_ModuleM4::refuse_slow_binding(const uint8_t *buf, uint16_t len)
{
    ModuleM4::BindingCallHeader call;
    if (!ModuleM4::parse_binding_call(buf, len, call)) {
        return false;
    }
    if (_server.frame_is_posted(buf, len)) {
        _post_drops++;   // a posted frame gets no BUSY reply: the M4 already answered OK
        return true;
    }
    _requests++;
    _session_calls++;
    _slow_refused++;
    send_frame(_tx_frame, ModuleM4::pack_binding_reply(_tx_frame, sizeof(_tx_frame), call.header.seq,
                                                     ModuleM4::ReplyStatus::Ok, ModuleM4::STATUS_BUSY, nullptr, 0,
                                                     _wake_latency_us, 0));
    if (_probing) {
        // unrun: no dispatch, lock or adapter stages
        const uint32_t now_us = CallLatencyLog::clock_us();
        const AP_ModuleM4_Server::ServeProbe unrun { now_us, now_us, now_us, now_us, 0, 0, 0, 0, 0, 0 };
        probe_served(buf, len, unrun, AP_Module::CALL_REFUSED);
    }
    return true;
}

static_assert(ModuleM4::ServeDutyCap::MAX_PAUSE_US <= ModuleM4::CALL_TIMEOUT_MS * 1000U * 3U / 5U,
              "a duty-cap pause leaves the M4's next call time to be served");

// listening: the ring was empty with the asleep flag set, so the M4 rings for its next frame
void AP_ModuleM4::pause_for_duty_cap(uint32_t pause_us, bool listening)
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
AP_ModuleM4::ServeResult AP_ModuleM4::serve()
{
    if (_state != State::Running) {
        return ServeResult::Stuck;   // frames from before a (re)start are dropped by the re-handshake
    }
    const ModuleM4::PassEnd end = ModuleM4::serve_pass(
        _rx_frame,
        [this](uint16_t &len) { return _channel->receive(_rx_frame, sizeof(_rx_frame), &len); },
        [this](uint16_t len) {
            // a frame the worker's posted batch handed back is dispatched like any other
            while (len != 0) {
                const bool fast = _server.frame_on_fast_lane(_rx_frame, len);
                if (_probing) {
                    const uint32_t start_cycles = CallLatencyLog::cycles();
                    _frame_received_us = CallLatencyLog::clock_us();
                    _frame_flags = uint16_t(fast ? AP_Module::CallLane::Fast : AP_Module::CallLane::Slow) |
                                   uint16_t(MIN(_pass_frames, uint8_t(15)) << AP_Module::CALL_POSITION_SHIFT);
                    _pass_frames = uint8_t(MIN(_pass_frames + 1, 255));
                    _frame_overhead_cycles = CallLatencyLog::cycles() - start_cycles;
                }
                if (fast) {
                    handle_message(_rx_frame, len);
                    len = 0;
                } else if (_duty_cap.refuses_slow_bindings() && refuse_slow_binding(_rx_frame, len)) {
                    len = 0;
                } else {
                    serve_on_worker(len);
                    len = _worker_handback_len;
                    _worker_handback_len = 0;
                }
            }
        },
        [this]() { return AP_HAL::micros() - _woke_us; });
    switch (end) {
    case ModuleM4::PassEnd::Idle:
        return ServeResult::Idle;
    case ModuleM4::PassEnd::ReceiveError:
        // -EBADMSG: corrupt or desynchronised ring; -EMSGSIZE: an oversized frame stays queued
        _protocol_errors++;
        return ServeResult::Stuck;
    case ModuleM4::PassEnd::Busy:
        break;
    }
    return ServeResult::Busy;
}

void AP_ModuleM4::handle_message(const uint8_t *buf, uint16_t len)
{
    ModuleM4::MsgHeader header;
    if (!ModuleM4::parse_header(buf, len, header)) {
        _protocol_errors++;   // an unknown frame type or a malformed frame
        return;
    }
    _requests++;
    switch (MsgType(header.type)) {
    case MsgType::Stats:
        log_rpc_stats(buf);
        return;
    case MsgType::Counters:
        take_counters(buf);
        return;
    case MsgType::BindingCall:
        serve_binding(buf, len);
        return;
    case MsgType::SystemCall:
        serve_system(buf, len);
        return;
    case MsgType::FaultInject:
    case MsgType::Ready:
    case MsgType::BindingReply:
    case MsgType::SystemReply:
    case MsgType::Control:
    case MsgType::ParameterChanged:
        _protocol_errors++;   // M7 -> M4 only
        return;
    }
}

void AP_ModuleM4::serve_binding(const uint8_t *buf, uint16_t len)
{
    _session_calls++;
    AP_ModuleM4_Server::ServeProbe served {};
    const uint16_t reply_len = _server.serve_binding(buf, len, _tx_frame, sizeof(_tx_frame), AP_HAL::millis(),
                                                     _probing ? &served : nullptr);
    if (_server.last_call_wrote_parameters()) {
        // the module reads its own write at once: the values go ahead of the reply, on the same ring
        _server.mark_all_live();
        send_mirror_changes();
    }
    switch (_server.last_posted_outcome()) {
    case AP_ModuleM4_Server::PostedOutcome::NotPosted:
        break;
    case AP_ModuleM4_Server::PostedOutcome::Served:
        return;
    case AP_ModuleM4_Server::PostedOutcome::Failed:
        _post_errors++;
        if (!_post_error_reported) {
            _post_error_reported = true;
            const int16_t index = _server.last_call_module();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: %.16s logger.write failed",
                          index >= 0 && index < _module_count ? _modules[index].name : "module");
        }
        return;
    case AP_ModuleM4_Server::PostedOutcome::Dropped:
        _post_drops++;
        return;
    case AP_ModuleM4_Server::PostedOutcome::Refused:
        _protocol_errors++;
        return;
    }
    const int16_t module = _server.last_call_module();
    const bool resync = module >= 0 && _server.take_resync(uint16_t(module));
    if (reply_len >= sizeof(ModuleM4::BindingReplyHeader)) {
        ModuleM4::BindingReplyHeader reply;
        memcpy(&reply, _tx_frame, sizeof(reply));
        reply.m7_wake_us = _wake_latency_us;
        if (resync) {
            reply.reserved2 |= ModuleM4::REPLY_FLAG_MIRROR_RESYNC;
        }
        memcpy(_tx_frame, &reply, sizeof(reply));
    }
    if (!send_frame(_tx_frame, reply_len) && resync) {
        _server.owe_resync(uint16_t(module));
    }
    if (_probing) {
        probe_served(buf, len, served, served.flags);
    }
}

void AP_ModuleM4::probe_served(const uint8_t *buf, uint16_t len, const AP_ModuleM4_Server::ServeProbe &served,
                             uint16_t flags)
{
    const uint32_t reply_sent_us = CallLatencyLog::clock_us();
    const uint32_t start_cycles = CallLatencyLog::cycles();
    ModuleM4::BindingCallHeader call;
    ModuleM4::CallProbeTrailer trailer;
    if (!ModuleM4::parse_binding_call(buf, len, call) || !ModuleM4::binding_call_trailer(buf, len, call, trailer)) {
        return;   // the M4 does not probe this session, or the trailer did not fit the frame
    }
    CallLatencyLog &log = CallLatencyLog::get();
    log.set_m4_stamp_cycles(trailer.stamp_cycles);
    log.set_m4_memset(trailer.memset_small_centicycles, trailer.memset_large_centicycles);
    AP_Module::CallTrace trace;
    // the M4's stamps of its previous probed call, which the M7 holds
    const AP_Module::M4Completion previous {
        trailer.prev_seq, trailer.prev_sent_us, trailer.prev_reply_seen_us, trailer.prev_return_us,
        uint16_t(trailer.prev_flags & PROBE_M4_FLAGS), uint32_t(trailer.prev_stamps) & AP_Module::CALL_STAMPS_M4,
        trailer.prev_packed_us, trailer.prev_posted_us, trailer.prev_forward_returned_us, trailer.prev_reply_isr_us,
        trailer.prev_reply_wake_us, trailer.prev_signalled_us, trailer.prev_resumed_us
    };
    AP_Module::CallStamps completed;
    if (_probe_pending.complete(previous, trace, &completed)) {
        log.record(trace);
        log.record_substages(trace.id, completed);
        const uint16_t pre_cycles[AP_Module::M4_PRE_PARTS] {
            trailer.prev_svc_to_entry_cycles, trailer.prev_checks_cycles, trailer.prev_begin_probe_cycles,
            trailer.prev_pack_cycles
        };
        log.record_m4_pre_cycles(trace.id, pre_cycles);
    }
    AP_Module::CallStamps stamps { trailer.svc_entry_us, trailer.pre_send_us, 0, _pass_us, _frame_received_us,
                                 served.dispatch, served.lock_wait, served.adapter, served.adapter_done,
                                 reply_sent_us, 0, 0 };
    stamps.chk_parsed = served.chk_parsed;
    stamps.chk_quota = served.chk_quota;
    stamps.chk_copied = served.chk_copied;
    stamps.taken = served.taken;
    // the doorbell belongs to the first frame of the pass it woke
    if (_pass_doorbell && (_frame_flags >> AP_Module::CALL_POSITION_SHIFT) == 0) {
        stamps.m7_doorbell = _pass_doorbell_us;
        stamps.taken |= AP_Module::CALL_STAMP_M7_DOORBELL;
    }
    AP_Module::CallTrace partial {};
    partial.time_us = AP_HAL::micros64();
    partial.seq = call.header.seq;
    partial.id = uint16_t(MIN(call.id, uint32_t(UINT16_MAX)));
    partial.flags = uint16_t(flags | _frame_flags | (_pass_cap_paused ? AP_Module::CALL_CAP_PAUSED : 0));
    if (_probe_pending.hold(partial, stamps, trace)) {
        log.record(trace);
    }
    log.add_overhead(_frame_overhead_cycles + served.overhead_cycles + (CallLatencyLog::cycles() - start_cycles));
}

// system calls: a module's text notify; a call of a method this M7 does not serve is answered
// UnknownMethod at once, so the M4 does not wait for its timeout
void AP_ModuleM4::serve_system(const uint8_t *buf, uint16_t len)
{
    ModuleM4::SystemCallHeader call;
    memcpy(&call, buf, sizeof(call));   // parse_header() checked len >= sizeof(SystemCallHeader)
    if ((call.flags & ModuleM4::CALL_FLAG_NOTIFY) != 0) {
        if (call.method == uint16_t(ModuleM4::SystemMethod::Text)) {
            send_module_text(buf, len);
        } else if (call.method == uint16_t(ModuleM4::SystemMethod::ModuleState)) {
            module_state(buf, len);
        }
        return;
    }
    _session_calls++;
    const ModuleM4::SystemMethod method = ModuleM4::SystemMethod(call.method);
    uint16_t reply_len;
    switch (method) {
    case ModuleM4::SystemMethod::Config:
        reply_len = serve_config(buf, len, call.header.seq);
        break;
    case ModuleM4::SystemMethod::ListModules:
        reply_len = serve_list_modules(call.header.seq);
        break;
    case ModuleM4::SystemMethod::Resolve:
        reply_len = serve_resolve(buf, len, call.header.seq);
        break;
    case ModuleM4::SystemMethod::FsOpen:
    case ModuleM4::SystemMethod::FsRead:
    case ModuleM4::SystemMethod::FsSeek:
    case ModuleM4::SystemMethod::FsClose:
        reply_len = serve_file(method, buf, len, call.header.seq);
        break;
    default:
        reply_len = pack_system_reply(method, call.header.seq, ModuleM4::ReplyStatus::UnknownMethod, -ENOSYS,
                                      sizeof(ModuleM4::SystemReplyHeader));
        break;
    }
    send_frame(_tx_frame, reply_len);
}

// the reply's header at the start of _tx_frame; the method's contents follow it there
uint16_t AP_ModuleM4::pack_system_reply(ModuleM4::SystemMethod method, uint32_t seq, ModuleM4::ReplyStatus transport,
                                      int32_t result, uint16_t length)
{
    ModuleM4::SystemReplyHeader reply {};
    ModuleM4::fill_header(reply.header, MsgType::SystemReply, length, seq);
    reply.method = uint16_t(method);
    reply.transport = uint8_t(transport);
    reply.result = result;
    memcpy(_tx_frame, &reply, sizeof(reply));
    return length;
}

/*
  The M4's Config, once per session: the module limits, the M7 clock, and the M4's CRC self-check,
  which the M7 checks. A CRC unit that disagrees with zlib's values cannot check what it loads:
  no module loads until reboot (ListModules answers none) and arming is refused
 */
uint16_t AP_ModuleM4::serve_config(const uint8_t *buf, uint16_t len, uint32_t seq)
{
    ModuleM4::ConfigRequest request;
    if (len != sizeof(request)) {
        return pack_system_reply(ModuleM4::SystemMethod::Config, seq, ModuleM4::ReplyStatus::Malformed, -EINVAL,
                                 sizeof(ModuleM4::SystemReplyHeader));
    }
    memcpy(&request, buf, sizeof(request));
    _config_seen = true;
    const ModuleM4::CrcSelfCheck check { request.crc_short, request.crc_pattern };
    if (!check.passed()) {
        _crc_failed = true;
        if (!_crc_failed_sent) {
            _crc_failed_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Module: M4 CRC unit check failed");
        }
    }
    if ((uint8_t(_options.get()) & OPTION_CRC_VALUES) != 0 && !_crc_values_sent) {
        _crc_values_sent = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: M4 CRC %08X %08X", unsigned(check.short_check),
                      unsigned(check.pattern_check));
    }
    ModuleM4::ConfigReply reply {};
    const AP_Module::Runtime *runtime = AP_Module::Runtime::get_singleton();
    if (runtime != nullptr) {
        const AP_Module::Runtime::Config config = runtime->config();
        reply.heap_max = config.heap_max;
        reply.stack_max = config.stack_max;
        reply.max_modules = uint8_t(config.max_modules);
        reply.debug_options = uint8_t(config.debug_options);
        // the PR rounds MOD_CODE_MAXSIZE up to whole 4096 B; the staged loader takes at most its cap,
        // at which any module within MOD_STACK_MAX and MOD_HEAP_MAX fits
        const uint32_t code_size = config.code_maxsize > 0 ? (uint32_t(config.code_maxsize) + 4095U) & ~4095U : 0;
        const uint32_t cap = ModuleM4::staged_code_size_cap(AP_MODULE_M4_MODULE_AREA_SIZE,
                                                          uint32_t(MAX(config.stack_max, int32_t(0))),
                                                          uint32_t(MAX(config.heap_max, int32_t(0))));
        reply.code_maxsize = int32_t(MIN(code_size, cap));
        if (code_size > cap && !_code_size_capped_sent) {
            _code_size_capped_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: MOD_CODE_MAXSIZE capped at %uKB on M4", unsigned(cap / 1024U));
        }
    }
    reply.m4_options = uint8_t(_options.get());
    reply.m7_millis = AP_HAL::millis();
    reply.m7_micros = AP_HAL::micros();
    const uint16_t length = pack_system_reply(ModuleM4::SystemMethod::Config, seq, ModuleM4::ReplyStatus::Ok, 0,
                                              sizeof(reply));
    memcpy(&reply.reply, _tx_frame, sizeof(reply.reply));
    memcpy(_tx_frame, &reply, sizeof(reply));
    return length;
}

// this session's discovery; none before its Config has shown a CRC unit that agrees with zlib's.
// Built in _tx_frame: the reply is too large for the worker's stack frame
uint16_t AP_ModuleM4::serve_list_modules(uint32_t seq)
{
    using ModuleM4::ListModulesReply;
    using ModuleM4::ModuleEntry;
    static_assert(sizeof(ListModulesReply) <= sizeof(_tx_frame), "the reply fits");
    static_assert(sizeof(ListModulesReply::modules) / sizeof(ModuleEntry) >= ModuleM4::MAX_MODULES, "every module listed");
    static_assert(sizeof(ModuleEntry::name) == sizeof(ModuleM4::ModuleFile::name) &&
                  sizeof(ModuleEntry::path) == sizeof(ModuleM4::ModuleFile::path), "names and paths fit");
    const uint8_t count = (_config_seen && !_crc_failed) ? _module_count : 0;
    memset(_tx_frame, 0, sizeof(ListModulesReply));
    for (uint8_t i = 0; i < count; i++) {
        uint8_t *entry = _tx_frame + offsetof(ListModulesReply, modules) + i * sizeof(ModuleEntry);
        memcpy(entry + offsetof(ModuleEntry, name), _modules[i].name, sizeof(ModuleEntry::name));
        memcpy(entry + offsetof(ModuleEntry, path), _modules[i].path, sizeof(ModuleEntry::path));
        memcpy(entry + offsetof(ModuleEntry, size), &_modules[i].size, sizeof(ModuleEntry::size));
    }
    _tx_frame[offsetof(ListModulesReply, count)] = count;
    return pack_system_reply(ModuleM4::SystemMethod::ListModules, seq, ModuleM4::ReplyStatus::Ok, count,
                             sizeof(ListModulesReply));
}

// module files, read only and in order (AP_ModuleM4_FileServer); the request is untrusted
uint16_t AP_ModuleM4::serve_file(ModuleM4::SystemMethod method, const uint8_t *buf, uint16_t len, uint32_t seq)
{
    const uint16_t header = sizeof(ModuleM4::SystemReplyHeader);
    switch (method) {
    case ModuleM4::SystemMethod::FsOpen: {
        ModuleM4::FsOpenRequest request;
        if (len != sizeof(request)) {
            break;
        }
        memcpy(&request, buf, sizeof(request));
        // one byte past the M4's: a path without a terminator in it is too long
        char path[sizeof(request.path) + 1];
        memcpy(path, request.path, sizeof(request.path));
        path[sizeof(request.path)] = 0;
        return pack_system_reply(method, seq, ModuleM4::ReplyStatus::Ok,
                                 _file_server.open(path, ModuleM4::MODULE_OPEN_FLAGS), header);
    }
    case ModuleM4::SystemMethod::FsRead: {
        ModuleM4::FsReadRequest request;
        if (len != sizeof(request)) {
            break;
        }
        memcpy(&request, buf, sizeof(request));
        static_assert(sizeof(ModuleM4::SystemReplyHeader) + ModuleM4::FS_READ_MAX <= sizeof(_tx_frame), "read fits");
        const int32_t n = _file_server.read(request.fd, _tx_frame + header, request.count);
        return pack_system_reply(method, seq, ModuleM4::ReplyStatus::Ok, n, header + uint16_t(MAX(n, int32_t(0))));
    }
    case ModuleM4::SystemMethod::FsSeek: {
        ModuleM4::FsSeekRequest request;
        if (len != sizeof(request)) {
            break;
        }
        memcpy(&request, buf, sizeof(request));
        return pack_system_reply(method, seq, ModuleM4::ReplyStatus::Ok,
                                 _file_server.seek(request.fd, request.offset), header);
    }
    case ModuleM4::SystemMethod::FsClose: {
        ModuleM4::FsCloseRequest request;
        if (len != sizeof(request)) {
            break;
        }
        memcpy(&request, buf, sizeof(request));
        const AP_ModuleM4_FileServer::Closed closed = _file_server.close(request.fd);
        if (closed.result == 0) {
            // the module's checksum is what this M7 served, never what the M4 reports
            _served_crc = closed.crc;
        }
        ModuleM4::FsCloseReply reply {};
        pack_system_reply(method, seq, ModuleM4::ReplyStatus::Ok, closed.result, sizeof(reply));
        memcpy(&reply.reply, _tx_frame, sizeof(reply.reply));
        reply.crc = closed.crc;
        reply.served = closed.served;
        memcpy(_tx_frame, &reply, sizeof(reply));
        return sizeof(reply);
    }
    default:
        break;
    }
    return pack_system_reply(method, seq, ModuleM4::ReplyStatus::Malformed, -EINVAL, header);
}

// a reply to the M4; len 0 (it did not pack) sends nothing
bool AP_ModuleM4::send_frame(const uint8_t *buf, uint16_t len)
{
    if (len == 0) {
        _dropped++;
        return false;
    }
    if (_channel->send(buf, len) != 0) {
        _dropped++;   // the M4 times the call out
        return false;
    }
    return true;
}

void AP_ModuleM4::send_ready(uint32_t now_ms)
{
    ModuleM4::ReadyMsg ready;
    // seq carries the session: the M4 drops a call outstanding across a session change
    ModuleM4::fill_header(ready.header, MsgType::Ready, sizeof(ready), _m4_session);
    _ready_sent_ms = now_ms;
    if (_channel->send(&ready, sizeof(ready)) != 0) {
        _dropped++;
    }
}

void AP_ModuleM4::send_module_text(const uint8_t *buf, uint16_t len)
{
    ModuleM4::TextNotify text;
    if (len != sizeof(text)) {
        return;
    }
    memcpy(&text, buf, sizeof(text));
    if (text.call.method != uint16_t(ModuleM4::SystemMethod::Text) || (text.call.flags & ModuleM4::CALL_FLAG_NOTIFY) == 0 ||
        text.severity > 7 || text.length > sizeof(text.text)) {
        return;
    }
    // the PR's Runtime::text: 44 characters, 8 queued, the rest dropped and counted; untrusted:
    // printable only
    WITH_SEMAPHORE(_text_sem);
    if (_text_count == ARRAY_SIZE(_texts)) {
        _texts_dropped++;
        return;
    }
    ModuleText &entry = _texts[(_text_head + _text_count) % ARRAY_SIZE(_texts)];
    entry.severity = text.severity;
    const uint16_t count = MIN(text.length, uint16_t(sizeof(entry.text) - 1));
    for (uint16_t i = 0; i < count; i++) {
        const char c = text.text[i];
        entry.text[i] = (c >= 0x20 && c < 0x7F) ? c : '?';
    }
    entry.text[count] = 0;
    _text_count++;
}

// the PR's Runtime::drain_messages, on the host thread: at most one queue's worth per pass
void AP_ModuleM4::drain_module_texts()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(_texts); i++) {
        ModuleText message;
        {
            WITH_SEMAPHORE(_text_sem);
            if (_text_count == 0) {
                break;
            }
            message = _texts[_text_head];
            _text_head = (_text_head + 1) % ARRAY_SIZE(_texts);
            _text_count--;
        }
        GCS_SEND_TEXT(MAV_SEVERITY(message.severity), "Module: %s", message.text);
    }
    uint32_t dropped;
    {
        WITH_SEMAPHORE(_text_sem);
        dropped = _texts_dropped;
        _texts_dropped = 0;
    }
    if (dropped != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: dropped %lu text messages", (unsigned long)dropped);
    }
}

// the M4 loader's import round: each hash resolved by the PR's binding_resolve(), UINT32_MAX for
// none. Built in _tx_frame
uint16_t AP_ModuleM4::serve_resolve(const uint8_t *buf, uint16_t len, uint32_t seq)
{
    using ModuleM4::ResolveReply;
    using ModuleM4::ResolveRequest;
    uint16_t count = 0;
    if (len >= offsetof(ResolveRequest, hashes)) {
        memcpy(&count, buf + offsetof(ResolveRequest, count), sizeof(count));
    }
    if (len < offsetof(ResolveRequest, hashes) || count > ModuleM4::RESOLVE_MAX ||
        len != offsetof(ResolveRequest, hashes) + uint32_t(count) * sizeof(uint64_t)) {
        return pack_system_reply(ModuleM4::SystemMethod::Resolve, seq, ModuleM4::ReplyStatus::Malformed, -EINVAL,
                                 sizeof(ModuleM4::SystemReplyHeader));
    }
    static_assert(sizeof(ResolveReply) <= sizeof(_tx_frame), "the reply fits");
    for (uint16_t i = 0; i < count; i++) {
        uint64_t hash;
        memcpy(&hash, buf + offsetof(ResolveRequest, hashes) + i * sizeof(hash), sizeof(hash));
        const uint32_t id = AP_Module::binding_resolve(nullptr, hash);
        memcpy(_tx_frame + offsetof(ResolveReply, ids) + i * sizeof(id), &id, sizeof(id));
    }
    memcpy(_tx_frame + offsetof(ResolveReply, count), &count, sizeof(count));
    memset(_tx_frame + offsetof(ResolveReply, reserved), 0, sizeof(ResolveReply::reserved));
    return pack_system_reply(ModuleM4::SystemMethod::Resolve, seq, ModuleM4::ReplyStatus::Ok, 0,
                             uint16_t(offsetof(ResolveReply, ids) + uint32_t(count) * sizeof(uint32_t)));
}

// the M4 loader's and module thread's reports. Loaded counts toward the PR's "loaded N modules" and
// its checksum toward MOD_LD_CSUM, Started toward MOD_RUN_CSUM until the module returns, panics or
// fails; a failure or a panic is the PR's fail(): "Module: <reason>", kept for the pre-arm check
void AP_ModuleM4::module_state(const uint8_t *buf, uint16_t len)
{
    ModuleM4::ModuleStateNotify notify;
    if (len != sizeof(notify)) {
        return;
    }
    memcpy(&notify, buf, sizeof(notify));
    if (notify.module >= _module_count) {
        return;
    }
    const uint8_t index = uint8_t(notify.module);
    const char *name = _modules[index].name;
    // untrusted: terminated, printable only
    notify.text[sizeof(notify.text) - 1] = 0;
    for (char *c = notify.text; *c != 0; c++) {
        if (*c < 0x20 || *c >= 0x7F) {
            *c = '?';
        }
    }
    char reason[80];
    switch (ModuleM4::ModuleStateKind(notify.state)) {
    case ModuleM4::ModuleStateKind::Loaded:
        if (_module_loaded[index]) {
            break;   // once per session
        }
        _module_loaded[index] = true;
        _module_crc[index] = _served_crc;
        _loaded_checksum ^= _module_crc[index];
        if (index == 0) {
            _module_rx_base = notify.rx_base;
            _module_rx_size = notify.rx_size;
            _module_stack_base = notify.stack_base;
        }
        if (++_modules_loaded == _module_count) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: loaded %u modules", unsigned(_module_count));
        }
        break;
    case ModuleM4::ModuleStateKind::Started:
        if (!_module_loaded[index] || _module_running[index]) {
            break;
        }
        _module_running[index] = true;
        _server.snapshot_module(index);
        _running_checksum ^= _module_crc[index];
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: started %s", name);
        break;
    case ModuleM4::ModuleStateKind::Failed:
        IGNORE_RETURN(module_ended(index));
        // the load's file, if it failed with the file open
        _file_server.reset();
        load_error(notify.text);
        break;
    case ModuleM4::ModuleStateKind::Returned:
        if (!module_ended(index)) {
            break;
        }
        if (notify.result != ModuleM4::STATUS_OK && notify.result != ModuleM4::STATUS_STOPPED) {
            hal.util->snprintf(reason, sizeof(reason), "%.31s returned %ld", name, long(notify.result));
            load_error(reason);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: exited %s", name);
        break;
    case ModuleM4::ModuleStateKind::Panicked:
        if (!module_ended(index)) {
            break;
        }
        // the PR's latch, which there needs a reboot: here a MOD_RESTART change reclaims the
        // parked module with a requested M4 restart
        hal.util->snprintf(reason, sizeof(reason), "%.19s panic; set MOD_RESTART", name);
        load_error(reason);
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Module fatal: %.36s", notify.text);
        break;
    case ModuleM4::ModuleStateKind::HeapNeeded:
        // the PR's report of an armed heap expansion, which the M4 heap cannot make
        if (!_module_loaded[index] || notify.heap_size <= _heap_needed[index]) {
            break;
        }
        _heap_needed[index] = notify.heap_size;
        _heap_required = MAX(_heap_required, notify.heap_size);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: %.24s initial heap needs %u", name, unsigned(notify.heap_size));
        break;
    }
}

// a running module of this session ended: false if it was not running
bool AP_ModuleM4::module_ended(uint8_t index)
{
    if (!_module_running[index]) {
        return false;
    }
    _module_running[index] = false;
    _running_checksum ^= _module_crc[index];
    return true;
}

void AP_ModuleM4::clear_modules()
{
    _post_error_reported = false;
    memset(_module_loaded, 0, sizeof(_module_loaded));
    memset(_module_running, 0, sizeof(_module_running));
    memset(_module_crc, 0, sizeof(_module_crc));
    memset(_heap_needed, 0, sizeof(_heap_needed));
    _heap_required = 0;
    _served_crc = 0;
    _loaded_checksum = 0;
    _running_checksum = 0;
    _module_rx_base = 0;
    _module_rx_size = 0;
    _module_stack_base = 0;
}

void AP_ModuleM4::checksums(uint32_t &loaded, uint32_t &running) const
{
    loaded = _loaded_checksum & CHECKSUM_MASK;
    running = _running_checksum & CHECKSUM_MASK;
}

// one row per method; SemMax carries the largest M7-side time of one call
void AP_ModuleM4::log_rpc_stats(const uint8_t *buf)
{
    ModuleM4::StatsMsg stats;
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
    // @Description: M4 module binding call round trips of one method, once per second (M4 builds)
    // @Field: TimeUS: Time since system startup
    // @Field: Id: method id
    // @Field: Name: method name, as the M4 reports it
    // @Field: N: round trips completed in the last second
    // @Field: P50: median round trip (log2 bucket bound)
    // @Field: P99: 99th percentile round trip (log2 bucket bound)
    // @Field: Max: longest round trip
    // @Field: SemMax: longest M7-side time of one call
    // @Field: Tot: total round-trip time
    // @Field: Post: 1 for a posted binding (the round trip is the M4's post-to-ring time)
    AP::logger().WriteStreaming("SRPC", "TimeUS,Id,Name,N,P50,P99,Max,SemMax,Tot,Post",
                                "s#--sssss-", "F---FFFFF-", "QHZIIIIIQB",
                                AP_HAL::micros64(), stats.method, name,
                                stats.count, stats.p50_us, stats.p99_us, stats.max_us, stats.m7_max_us,
                                uint64_t(stats.total_us), uint8_t(stats.reserved & ModuleM4::STATS_ROW_POSTED));
#endif
}

void AP_ModuleM4::take_counters(const uint8_t *buf)
{
    ModuleM4::CountersMsg counters;
    memcpy(&counters, buf, sizeof(counters));   // parse_header() checked the length
    _m4_mirror_hits += counters.mirror_hits;
    _m4_mirror_misses += counters.mirror_misses;
    _m4_post_drops += counters.post_drops;
}

// @LoggerMessage: M4ST
// @Description: M4 module supervisor, once per second (M4 builds)
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
// @Field: QHit: binding calls refused by their quota since the previous row (answered OK unrun: a bool result reads false)
// @Field: Cap: pauses the host's duty cap (10 % of the M7's CPU) forced in the last second
// @Field: CPU: host thread and slow worker CPU time in the last second, per mille of the M7 (what the duty cap charges)
// @Field: Lng: host passes over 50 ms of wall time in the last second (blocked in a binding, or preempted)
void AP_ModuleM4::log_supervisor(uint32_t now_ms, uint32_t elapsed_ms)
{
#if HAL_LOGGING_ENABLED
    AP::logger().WriteStreaming("M4ST", "TimeUS,St,HB,Ses,Rst,Flt,FPC,Req,PErr,Drop,Lost,QHit,Cap,CPU,Lng",
                                "s--------------", "F--------------", "QBIIBIIIHHIIHHH",
                                AP_HAL::micros64(), uint8_t(_state), _m4_heartbeat, _m4_session,
                                _restarts_used, _m4_fault_count, _m4_fault_pc, _requests,
                                _protocol_errors, _dropped, _m4_calls_lost, _server.take_quota_hits(),
                                _duty_cap_hits,
                                uint16_t(MIN(_host_cpu_us / MAX(elapsed_ms, 1U), uint32_t(UINT16_MAX))),
                                _long_passes);
    // @LoggerMessage: M4HP
    // @Description: M4 module host thread passes, once per second (M4 builds)
    // @Field: TimeUS: Time since system startup
    // @Field: PassMax: the largest CPU time of one fast-lane pass in the last second, in microseconds (what the duty cap charges)
    // @Field: WallBusy: host thread wall time from wake to the end of each pass in the last second, per mille (thread statistics do not feed it; preemption and waits on the slow worker count, the switch around a pass does not)
    // @Field: StatsReset: thread statistics resets seen in the last second, host and slow worker (an interval with one is charged the CPU since the reset plus 200 us, at most its wall time)
    // @Field: SlowReq: frames the slow worker served in the last second
    // @Field: SlowMax: the longest of them, wall time in microseconds
    // @Field: SlowCPU: the largest CPU time of one of them, in microseconds (charged to the duty cap with the host's)
    // @Field: Deny: binding calls for the slow worker refused unrun in the last second, the duty cap's debt being beyond 50 ms
    AP::logger().WriteStreaming("M4HP", "TimeUS,PassMax,WallBusy,StatsReset,SlowReq,SlowMax,SlowCPU,Deny", "s-------",
                                "F-------", "QIHHIIII", AP_HAL::micros64(), _pass_cpu_max_us,
                                uint16_t(MIN(_host_busy_us / MAX(elapsed_ms, 1U), uint32_t(UINT16_MAX))),
                                uint16_t(_cpu_charge.take_resets() + _worker_cpu_charge.take_resets()), _slow_requests,
                                _slow_max_us, _slow_cpu_max_us, _slow_refused);
    // @LoggerMessage: M4MR
    // @Description: M4 module Parameter mirror and posted calls, once per second (M4 builds)
    // @Field: TimeUS: Time since system startup
    // @Field: MirHit: Parameter.get answered on the M4 in the last M4 stats window
    // @Field: MirMiss: Parameter.get forwarded to the M7 in the last M4 stats window
    // @Field: MirTx: ParameterChanged frames sent in the last second
    // @Field: MirEnt: Parameter values those frames carried
    // @Field: MirRsy: ParameterChanged frames that did not fit the ring (the module's gets are forwarded until resent)
    // @Field: PErr: posted calls the M7 ran with a status other than OK
    // @Field: PDrop: posted calls dropped (M4 timeout, quota or duty-cap refusal)
    // @Field: PBat: posted calls the slow worker served in a batch after the one handed to it
    AP::logger().WriteStreaming("M4MR", "TimeUS,MirHit,MirMiss,MirTx,MirEnt,MirRsy,PErr,PDrop,PBat",
                                "s--------", "F--------", "QIIIIIIII", AP_HAL::micros64(), _m4_mirror_hits,
                                _m4_mirror_misses, _mirror_frames_sent, _mirror_entries_sent, _mirror_failed_sends,
                                _post_errors, _post_drops + _m4_post_drops, _posted_batched);
#endif
    switch (_refusal_notice.update(now_ms, _slow_refused)) {
    case ModuleM4::RefusalNotice::Text::Refused:
        // within one 50-character STATUSTEXT
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: M4 calls refused, M7 duty cap %u/s",
                      unsigned(MIN(_slow_refused, 99999U)));
        break;
    case ModuleM4::RefusalNotice::Text::ServedAgain:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: M4 calls served again");
        break;
    case ModuleM4::RefusalNotice::Text::None:
        break;
    }
    _m4_calls_lost = 0;
    if ((uint8_t(_options.get()) & OPTION_RPC_SUMMARY) == 0 || _state != State::Running ||
        now_ms - _last_summary_ms < 10000) {
        return;
    }
    const uint32_t window_ms = _last_summary_ms != 0 ? now_ms - _last_summary_ms : 10000;
    _last_summary_ms = now_ms;
    const uint32_t per_second = uint32_t(uint64_t(_rpc_summary.count) * 1000 / MAX(window_ms, 1U));
    // clamped within one 50-character STATUSTEXT
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: RPC %u/s p50<%u p99<%u max %uus",
                  unsigned(MIN(per_second, 9999U)), unsigned(MIN(_rpc_summary.p50_us, 9999U)),
                  unsigned(MIN(_rpc_summary.p99_us, 9999U)), unsigned(MIN(_rpc_summary.max_us, 99999U)));
    memset(&_rpc_summary, 0, sizeof(_rpc_summary));
    // busy microseconds per millisecond = per mille of one core
    const uint32_t host_permille = _host_busy_us / MAX(elapsed_ms, 1U);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: M7 host %u.%u%% CPU",
                  unsigned(host_permille / 10), unsigned(host_permille % 10));
}

void AP_ModuleM4::supervise(uint32_t now_ms)
{
    switch (_state) {
    case State::Restarting:
        if (_grant_pending) {
            // a fault restart's backoff: the M4 waits for the grant (in its fault handler, or a SITL
            // M4 that exited is started then), and the restart timeout counts from the grant
            if (int32_t(platform_timeout_ms() - _grant_due_timeout_ms) >= 0) {
                grant_restart();
            }
            break;
        }
        check_started(now_ms);
        break;
    case State::Starting:
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
  An M4 fault record: a short name for the one-line restart text ("MemManage", in SITL "signal 11"),
  its detail ("MemManage PC=0x0810A3C4", ModuleM4::fault_reason_text; in SITL the signal and address)
  and, for a PC in the module's code, where in the module ("fault in airshow.apmod+0x1A2C")
 */
void AP_ModuleM4::report_fault(uint32_t cfsr, uint32_t hfsr, uint32_t pc, uint32_t lr)
{
    char name[16];
    char detail[80];
    char location[48] {};
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    (void)hfsr;
    (void)lr;
    hal.util->snprintf(name, sizeof(name), "signal %u", unsigned(cfsr));
    hal.util->snprintf(detail, sizeof(detail), "signal %u addr=0x%08X", unsigned(cfsr), unsigned(pc));
#else
    hal.util->snprintf(name, sizeof(name), "%s", ModuleM4::fault_name(cfsr, hfsr));
    ModuleM4::fault_reason_text(detail, sizeof(detail), cfsr, hfsr, pc, lr, _module_stack_base);
    if (_module_count != 0 && _module_rx_size != 0 && !ModuleM4::fault_on_stacking(cfsr) &&
        pc - _module_rx_base < _module_rx_size) {
        hal.util->snprintf(location, sizeof(location), "fault in %.22s+0x%X", _modules[0].name,
                           unsigned(pc - _module_rx_base));
    }
#endif
    failure(name, detail, location[0] != 0 ? location : nullptr);
}

void AP_ModuleM4::check_started(uint32_t now_ms)
{
    PeerHeader::View view {};
    const PeerHeader::Status status = PeerHeader::readPeer(_shared, PeerHeader::Core::M7,
                                                           ModuleM4::PROTOCOL_VERSION,
                                                           ModuleM4::CONTRACT_HASH, &view);
    if (status == PeerHeader::Status::Mismatch) {
        go_off("image mismatch");
        return;
    }
    if (status == PeerHeader::Status::Compatible && view.restarts == _restart_grants &&
        (_state == State::Starting || view.session != _m4_session)) {
        // this session's modules, found before it is Ready (the start text counts them); the M4
        // waits meanwhile, and the heartbeat is taken after
        _file_server.reset();
        discover_on_worker();
        if (_run_control.stopped()) {
            _module_count = 0;   // stopped (MOD_ENABLE 0): a session that loads no module
        }
        clear_modules();
        now_ms = AP_HAL::millis();
        _m4_session = view.session;
        // a fault restart has been charged: a fault the M4 recorded on its way down is the same
        // incident. A fault the new session recorded after its publish() and before this read
        // shows here too; check_running() reports it if the session never beats. After a requested
        // restart the count stays: a fault recorded around the request is still a fault, which
        // check_running() charges to MOD_M4_RST_MAX
        _start_fault.session_started(_state == State::Restarting && !_restart_requested,
                                     view.faultCount, _m4_fault_count);
        if (_state == State::Starting || !_restart_requested) {
            _m4_fault_count = view.faultCount;
        }
        _m4_heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
        _m4_heartbeat_ms = platform_timeout_ms();
        // re-handshake: drop whatever either side queued before this start
        _channel->resetAfterRestart();
        _session_calls = 0;
        _config_seen = false;
        _crc_failed_sent = false;
        _crc_values_sent = false;
        // the previous session's modules are gone: their slots end, through the PR's cleanups
        _server.reset(_m4_session, _module_count);
        _modules_loaded = 0;
        memset(&_rpc_summary, 0, sizeof(_rpc_summary));
        _restart_requested = false;
        enter(State::Running, now_ms);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: started on M4 (%u module%s, M4 RAM %uKB)", unsigned(_module_count),
                      _module_count == 1 ? "" : "s", unsigned(AP_MODULE_M4_MODULE_AREA_SIZE / 1024U));
#if AP_MODULE_M4_PRIORITY_TEST
        static bool priority_test_sent;
        if (!priority_test_sent) {
            priority_test_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: M4 host prio test (host 183, worker 182)");
        }
#endif
        // the M4 forwards module calls only after this
        send_ready(now_ms);
        return;
    }
    const uint32_t timeout_ms = _state == State::Starting ? ModuleM4::START_TIMEOUT_MS
                                                          : PLATFORM_RESTART_TIMEOUT_MS;
    if (platform_timeout_ms() - _state_start_timeout_ms > timeout_ms) {
        go_off("not responding");
    }
}

void AP_ModuleM4::check_running(uint32_t now_ms)
{
    if (platform_m4_exited()) {
        failure("process exit");
        return;
    }
    PeerHeader::View view {};
    const PeerHeader::Status status = PeerHeader::readPeer(_shared, PeerHeader::Core::M7,
                                                           ModuleM4::PROTOCOL_VERSION,
                                                           ModuleM4::CONTRACT_HASH, &view);
    if (status == PeerHeader::Status::Mismatch) {
        go_off("image mismatch");
        return;
    }
    if (status == PeerHeader::Status::Compatible) {
        if (view.faultCount != _m4_fault_count) {
            _m4_fault_count = view.faultCount;
            _m4_fault_pc = view.faultPc;
            report_fault(view.faultCfsr, view.faultHfsr, view.faultPc, view.faultLr);
            return;
        }
        if (view.session != _m4_session) {
            failure("self restart");
            return;
        }
    }
    // Busy: the M4 is rewriting its header (or died doing so); the heartbeat decides. The M4 beats
    // on its own clock: in SITL wall time, which SIM_SPEEDUP does not scale
    const uint32_t heartbeat = PeerHeader::peerHeartbeat(_shared, PeerHeader::Core::M7);
    const uint32_t beat_check_ms = platform_timeout_ms();
    if (heartbeat != _m4_heartbeat) {
        _m4_heartbeat = heartbeat;
        _m4_heartbeat_ms = beat_check_ms;
        _start_fault.beat();
    } else if (beat_check_ms - _m4_heartbeat_ms > ModuleM4::HEARTBEAT_TIMEOUT_MS) {
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
bool AP_ModuleM4::report_start_fault()
{
    PeerHeader::View view {};
    const bool readable = PeerHeader::readPeer(_shared, PeerHeader::Core::M7, ModuleM4::PROTOCOL_VERSION,
                                               ModuleM4::CONTRACT_HASH, &view) == PeerHeader::Status::Compatible;
    if (!_start_fault.report_on_timeout(readable)) {
        return false;
    }
    _m4_fault_pc = view.faultPc;
    report_fault(view.faultCfsr, view.faultHfsr, view.faultPc, view.faultLr);
    return true;
}

uint8_t AP_ModuleM4::restart_limit() const
{
    return uint8_t(constrain_int16(_restart_max.get(), 0, 10));
}

/*
  reason: short (at most 12 characters), so the restart text stays within one 50-character
  STATUSTEXT: "Module: M4 fault (process exit), restart (10/10)". detail: an M4 fault record's
  text, and location where in the module it faulted, each sent first on its own line. The restart
  is granted after restart_backoff_ms() of the restarts used (supervise())
 */
void AP_ModuleM4::failure(const char *reason, const char *detail, const char *location)
{
    if (detail != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Module: M4 %s", detail);
    }
    if (location != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Module: M4 %s", location);
    }
    const bool held_requests = _start_fault.take_held();
    if (_restarts_used >= restart_limit()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Module: M4 fault (%s)", reason);
        go_off("fault limit");
        return;
    }
    _restarts_used++;
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Module: M4 fault (%s), restart (%u/%u)", reason,
                  unsigned(_restarts_used), unsigned(restart_limit()));
    _restart_requested = false;   // a fault during a requested restart is a fault
    // the session's modules are gone now; the grant waits for the backoff
    _server.reset(0, 0);
    _file_server.reset();
    clear_modules();
    _stopping = false;
    _grant_pending = true;
    _grant_due_timeout_ms = platform_timeout_ms() + ModuleM4::restart_backoff_ms(_restarts_used);
    enter(State::Restarting, AP_HAL::millis());
    // a restart asked while a start fault was pending is served by this fault restart
    if (held_requests) {
        bool restart;
        IGNORE_RETURN(_run_requests.take(restart));
    }
}

// a restart for the M4, a fault's or a request's: the grant count is its own, so requested restarts
// spend none of MOD_M4_RST_MAX (_restarts_used)
void AP_ModuleM4::grant_restart()
{
    // the session's modules and files end with it
    _server.reset(0, 0);
    _file_server.reset();
    clear_modules();
    _stopping = false;
    _grant_pending = false;
    _restart_grants++;
    // the new session starts without the old one's protocol errors
    _protocol_errors = 0;
    // the M4 acts on a restart doorbell only once the grant has changed: store it first
    ModuleM4::supervisor_block(_shared)->restart_grant.store(_restart_grants);
    platform_request_restart();
    enter(State::Restarting, AP_HAL::millis());
}

// MOD_RESTART changes (update()), taken while a session runs: a request made while the M4 starts
// or restarts waits in _run_requests, and an M4 that is off is never restarted
void AP_ModuleM4::run_control(uint32_t now_ms)
{
    if (_stopping) {
        check_stopped();
        return;
    }
    // a fault at the session's start is pending: until the heartbeat tells whose it is (at most
    // HEARTBEAT_TIMEOUT_MS), a request waits, so it can never charge an absorbed fault again
    if (_start_fault.holds_requests()) {
        return;
    }
    if (take_run_request(now_ms - _state_start_ms) != ModuleM4::RunAction::None) {
        stop_modules();
    }
}

// the stop step of a requested restart: each running module is asked to stop (the M4 sets its stop
// flag and wakes its sleep); the restart follows once none runs, a returned or a panicked module
// being stopped, or after MODULE_STOP_TIMEOUT_MS
void AP_ModuleM4::stop_modules()
{
    bool running = false;
    for (uint8_t i = 0; i < _module_count; i++) {
        if (!_module_running[i]) {
            continue;
        }
        running = true;
        ModuleM4::ControlMsg control {};
        // seq carries the session: the M4 drops a stop of an older one
        ModuleM4::fill_header(control.header, MsgType::Control, sizeof(control), _m4_session);
        control.module = i;
        control.action = ModuleM4::CONTROL_STOP;
        if (_channel->send(&control, sizeof(control)) != 0) {
            _dropped++;   // the stop timeout reclaims the module
        }
    }
    if (!running) {
        request_restart();
        return;
    }
    _stopping = true;
    _stop_start_timeout_ms = platform_timeout_ms();
}

void AP_ModuleM4::check_stopped()
{
    bool running = false;
    for (uint8_t i = 0; i < _module_count; i++) {
        running = running || _module_running[i];
    }
    if (running && platform_timeout_ms() - _stop_start_timeout_ms < ModuleM4::MODULE_STOP_TIMEOUT_MS) {
        return;
    }
    for (uint8_t i = 0; i < _module_count; i++) {
        if (_module_running[i]) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: %.15s did not stop, M4 restarted", _modules[i].name);
        }
    }
    request_restart();
}

// take a restart request and say what it does; running_ms: the session's age
ModuleM4::RunAction AP_ModuleM4::take_run_request(uint32_t running_ms)
{
    bool restart = false;
    const bool asked = _run_requests.take(restart);
    return _run_control.update(_enabled, asked, restart, running_ms);
}

// a restart the M7 asks for: the session ends as after a fault, and the new one loads the modules
void AP_ModuleM4::request_restart()
{
    _restart_requested = true;
#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
    // MOD_M4_INJECT 12 marks the next grant for a fault restart only
    ModuleM4::supervisor_block(_shared)->fault_after_publish_grant.store(0);
#endif
    grant_restart();
}

void AP_ModuleM4::go_off(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Module: M4 %s, off until reboot", reason);
    if (_shared != nullptr) {
        ModuleM4::supervisor_block(_shared)->run_permit.store(0);
    }
    // after run_permit: stopping rings the restart doorbell, which the M4 reads as park
    platform_stop();
    _server.reset(0, 0);
    _file_server.reset();
    clear_modules();
    _stopping = false;
    _grant_pending = false;
    enter(State::Off, AP_HAL::millis());
}

void AP_ModuleM4::enter(State state, uint32_t now_ms)
{
    _state = state;
    _state_start_ms = now_ms;
    _state_start_timeout_ms = platform_timeout_ms();
}

bool AP_ModuleM4::arming_checks(size_t buflen, char *buffer, const AP_Module::Runtime::Config &config) const
{
    // as the in-process runtime: MOD_ENABLE 0 (Held: it was 0 at boot) or MOD_DEBUG_OPTS bit 4
    if (_state == State::Held || config.enable == 0 || (uint8_t(config.debug_options) & DEBUG_OPTION_NO_PREARM) != 0) {
        return true;
    }
    if (_state == State::Off) {
        hal.util->snprintf(buffer, buflen, "Module: M4 not running");
        return false;
    }
    if (_state != State::Running) {
        hal.util->snprintf(buffer, buflen, "Module: M4 modules not started");
        return false;
    }
    if (_crc_failed) {
        hal.util->snprintf(buffer, buflen, "Module: M4 CRC unit check failed");
        return false;
    }
    if (_load_error[0] != 0) {
        hal.util->snprintf(buffer, buflen, "%s", _load_error);
        return false;
    }
    if (_heap_required != 0) {
        hal.util->snprintf(buffer, buflen, "Module: rebuild with initial heap at least %u", unsigned(_heap_required));
        return false;
    }
    uint32_t loaded, running;
    checksums(loaded, running);
    if (config.loaded_required != -1 && (uint32_t(config.loaded_required) & CHECKSUM_MASK) != loaded) {
        hal.util->snprintf(buffer, buflen, "Module: loaded checksum mismatch");
        return false;
    }
    if (config.running_required != -1 && (uint32_t(config.running_required) & CHECKSUM_MASK) != running) {
        hal.util->snprintf(buffer, buflen, "Module: running checksum mismatch");
        return false;
    }
    return true;
}

#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
void AP_ModuleM4::send_fault_inject()
{
    const int8_t kind = _inject.get();
    if (kind == _inject_seen) {
        return;
    }
    _inject_seen = kind;
    if (kind <= 0 || _state != State::Running) {
        return;
    }
    if (kind == ModuleM4::INJECT_MODULE_SELFTEST && _module_count != 0) {
        // the selftest runs in the module area on the loader's link: only in a session without modules
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: M4 thread check needs 0 modules");
        return;
    }
    uint32_t sent;
    if (kind == ModuleM4::INJECT_FAULT_AFTER_PUBLISH) {
        // the fault restart this HardFault causes is grant _restart_grants + 1
        ModuleM4::supervisor_block(_shared)->fault_after_publish_grant.store(_restart_grants + 1);
        sent = uint32_t(ModuleM4::FaultKind::HardFault);
    } else if (kind == ModuleM4::INJECT_MODULE_SELFTEST ||
               (kind <= ModuleM4::FAULT_KIND_MAX && kind != ModuleM4::INJECT_REFUSE_BINDINGS &&
                !ModuleM4::module_fault_kind(kind))) {
        // the service kinds and the thread-path selftest
        sent = uint32_t(kind);
    } else {
        // the module kinds: the test module reads MOD_M4_INJECT itself
        return;
    }
    ModuleM4::FaultInjectMsg inject;
    ModuleM4::fill_header(inject.header, MsgType::FaultInject, sizeof(inject), 0);
    inject.kind = sent;
    if (_channel->send(&inject, sizeof(inject)) != 0) {
        _dropped++;
        // no HardFault, so no fault restart for the mark
        ModuleM4::supervisor_block(_shared)->fault_after_publish_grant.store(0);
    }
}
#endif

#endif  // AP_MODULE_M4_ENABLED
