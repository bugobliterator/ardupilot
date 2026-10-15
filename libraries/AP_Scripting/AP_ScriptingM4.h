#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include "AP_ScriptingM4_Protocol.h"
#include "AP_ScriptingM4_Server.h"
#include "AP_ScriptingM4_RunControl.h"
#include "AP_ScriptingM4_ServeWait.h"
#include "AP_ScriptingM4_StartFault.h"
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
#include "AP_ScriptingM4_NetRegionClear.h"
#include <atomic>
#endif

namespace CubePilotFW {
class Channel;
}

/*
  M7 side of Lua scripting on the Cortex-M4: boots the M4 image, serves its
  Lua calls (AP_ScriptingM4_Server) and supervises it (heartbeat, fault
  record, restart budget). The M7 never waits on the M4: shared-memory
  operations never block, ring-full is a counted drop, and the host thread
  waits only on its own doorbell semaphore, with a timeout.
 */
namespace ScriptingM4 {
// thread_create() offsets: the host on PRIORITY_UART, the slow worker on PRIORITY_IO (on ChibiOS
// 61 and 56; the bounds are checked in AP_ScriptingM4_ChibiOS.cpp)
static const int8_t HOST_PRIORITY_OFFSET = 1;
static const int8_t WORKER_PRIORITY_OFFSET = -2;
}

class AP_ScriptingM4 {
public:
    AP_ScriptingM4();
    CLASS_NO_COPY(AP_ScriptingM4);

    enum class State : uint8_t {
        Held = 0,         // not started
        Starting = 1,
        Running = 2,
        Restarting = 3,
        Off = 4,          // until reboot
    };

    // start the M4 and the host thread; called once from AP_Scripting::init()
    void init();
    // the core state, the last script error (as the in-process checks), then
    // whether this session's scripts have started
    bool arming_checks(size_t buflen, char *buffer) const;
    void set_summary_enabled(bool enabled) { _summary_enabled = enabled; }
    State state() const { return _state; }
    uint32_t loaded_checksum() const { return _server.loaded_checksum(); }
    uint32_t running_checksum() const { return _server.running_checksum(); }
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    // the session the M4 publishes in the network status block: the restart grant it runs under.
    // Read by AP_Networking's threads
    uint32_t m4_session() const { return _restart_grants.load(std::memory_order_acquire); }
    // the M4 runs session, a value m4_session() returned: its peer header reported that grant
    // (check_starting()). Until then the last session's M4 may still be running
    bool m4_session_confirmed(uint32_t session) const {
        return _confirmed_grant.load(std::memory_order_acquire) == session + 1U;
    }
    bool m4_session_confirmed() const { return m4_session_confirmed(m4_session()); }
#endif

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class ServeResult : uint8_t {
        Idle,     // the ring is empty
        Busy,     // the per-wake budget ran out with a frame just served; more may remain
        Stuck,    // nothing can be served now: not running, or a protocol error
    };

    AP_Int8 _restart_max;
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    AP_Int8 _inject;
    int8_t _inject_seen;
#endif

    void host_thread();
    void worker_thread();
    void pause_for_duty_cap(uint32_t pause_us, bool listening);
    ServeResult serve();
    bool on_fast_lane(const uint8_t *buf, uint16_t len) const;
    void serve_on_worker(uint16_t len);
    bool refuse_slow_binding(const uint8_t *buf, uint16_t len);
    void charge_cpu(bool pass_end);
    void handle_message(const uint8_t *buf, uint16_t len);
    void handle_call(const uint8_t *buf, uint16_t len, const ScriptingM4::MsgHeader &header, uint32_t start_us);
    void send_reply(const ScriptingM4::MsgHeader &header, ScriptingM4::ReplyStatus status,
                    ScriptingM4::RpcWriter &results, uint8_t error_arg, uint32_t start_us);
    void send_ready(uint32_t now_ms);
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    void send_net_config(uint32_t seq);
#endif
    void log_rpc_stats(const uint8_t *buf);
    void log_supervisor(uint32_t now_ms, uint32_t elapsed_ms);
    void supervise(uint32_t now_ms);
    void check_started(uint32_t now_ms);
    void check_running(uint32_t now_ms);
    void failure(const char *reason);
    bool report_start_fault();
    void go_off(const char *reason);
    void grant_restart();
    void run_control(uint32_t now_ms);
    ScriptingM4::RunAction take_run_request(uint32_t running_ms);
    void request_restart(bool stopped);
    void enter(State state, uint32_t now_ms);
    uint8_t restart_limit() const;
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    void send_fault_inject();
#endif

    // per platform: AP_ScriptingM4_ChibiOS.cpp, AP_ScriptingM4_SITL.cpp
    bool platform_start(char *reason, size_t reason_len);
    void platform_request_restart();
    bool platform_m4_exited();
    void platform_stop();
    uint32_t platform_wait_us() const;
    // CPU time this thread has used, in microseconds; true when the clock was reset (statistics
    // cleared) since the previous call, so the difference to that reading is not the CPU used
    bool platform_thread_cpu_us(uint64_t &cpu_us);
    // the clock the start and restart timeouts count: the vehicle's on ChibiOS, wall-clock time in
    // SITL, where the M4 process runs on wall-clock time whatever SIM_SPEEDUP is
    uint32_t platform_timeout_ms() const;

    static void doorbell_isr(uint8_t group);

    CubePilotFW::Channel *_channel;
    uint8_t *_shared;                 // the shared window
    HAL_BinarySemaphore _doorbell;
    volatile uint32_t _doorbell_us;   // when the last doorbell interrupt arrived, 0 once consumed
    State _state;
    uint32_t _state_start_ms;
    uint32_t _state_start_timeout_ms; // platform_timeout_ms() when the state was entered
    uint8_t _restarts_used;
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    std::atomic<uint32_t> _restart_grants;   // restarts granted the M4: fault restarts and requested ones
    std::atomic<uint32_t> _confirmed_grant;  // the grant the M4 reported starting, plus one; 0: none yet
#else
    uint32_t _restart_grants;         // restarts granted the M4: fault restarts and requested ones
#endif
    bool _restart_requested;          // the restart under way is a request, not a fault
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    // whether the next grant may clear the network region
    ScriptingM4::NetRegionClear _net_region_clear;
#endif
    ScriptingM4::RunControl _run_control;
    uint32_t _m4_session;             // M4 peer header session being served
    uint32_t _m4_fault_count;         // M4 fault count already handled
    uint32_t _m4_fault_pc;
    uint32_t _m4_heartbeat;
    uint32_t _m4_heartbeat_ms;
    // a fault at a fault restart's start: the old session's on its way down, or the new session's
    // own before this M7 read its header; reported if the session never beats (report_start_fault()).
    // Stop and restart requests wait while it is pending
    ScriptingM4::StartFault _start_fault;
    uint32_t _woke_us;
    uint32_t _wake_latency_us;

    AP_ScriptingM4_Server _server;
    uint8_t _rx_frame[ScriptingM4::MAX_FRAME];
    uint8_t _tx_frame[ScriptingM4::MAX_FRAME];
    uint32_t _session_calls;          // calls and notifies of the current M4 session
    uint32_t _ready_sent_ms;

    // counters of the current one-second window
    uint32_t _stats_start_ms;
    uint32_t _requests;
    uint16_t _protocol_errors;
    uint16_t _dropped;
    uint32_t _host_busy_us;
    uint16_t _duty_cap_hits;           // pauses the duty cap forced
    uint32_t _host_cpu_us;             // host and slow worker CPU time, what the duty cap charges
    uint16_t _long_passes;             // passes over LONG_PASS_US of wall time
    uint32_t _pass_cpu_max_us;         // the largest CPU time of one pass
    uint32_t _slow_requests;           // frames the slow worker served
    uint32_t _slow_max_us;             // the longest of them, wall time
    uint32_t _slow_cpu_max_us;         // the most CPU one of them used
    uint32_t _slow_refused;            // binding calls for the worker refused unrun: duty-cap debt too deep
    ScriptingM4::RefusalNotice _refusal_notice;   // GCS texts about those refusals
    bool _summary_enabled;
    // the host's share of the M7: it runs above the I/O threads, a flooding M4 must not starve them
    ScriptingM4::ServeDutyCap _duty_cap;
    // ChibiOS charges all the host's CPU (its waits sleep); SITL only its passes (its waits spin)
    ScriptingM4::HostCpuCharge _cpu_charge { CONFIG_HAL_BOARD != HAL_BOARD_SITL };

    // the slow worker, below the I/O threads, serves the frames the fast lane does not
    // (frame_on_fast_lane()): the host hands it one frame (_rx_frame) and waits for it, so the
    // M4 sees its replies in order and only one of the two threads runs the server at a time
    HAL_BinarySemaphore _worker_request;
    HAL_BinarySemaphore _worker_done;
    uint16_t _worker_len;
    // the worker's CPU per frame (pass only, as its waits are not the host's), read by the host
    // after _worker_done
    ScriptingM4::HostCpuCharge _worker_cpu_charge { false };
    uint32_t _worker_used_us;

    struct {
        uint32_t count;           // calls timed since the last summary
        uint32_t busiest_count;   // calls of the busiest method in one report
        uint32_t p50_us;          // that method's p50
        uint32_t p99_us;          // largest p99 reported
        uint32_t max_us;          // largest round trip reported
    } _rpc_summary;
    uint32_t _m4_calls_lost;      // calls without a reply since the last M4ST row
    uint32_t _last_summary_ms;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    int _m4_pid;
    bool spawn_m4(char *reason, size_t reason_len);
#endif

    static AP_ScriptingM4 *_singleton;
};

#endif  // AP_SCRIPTING_M4_ENABLED
