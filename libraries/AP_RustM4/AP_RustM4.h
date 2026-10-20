#pragma once

#include "AP_RustM4_config.h"

#if AP_RUST_M4_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_Rust/Runtime.h>
#include "AP_RustM4_Protocol.h"
#include "AP_RustM4_RunControl.h"
#include "AP_RustM4_ServeWait.h"
#include "AP_RustM4_StartFault.h"

namespace CubePilotFW {
class Channel;
}

/*
  M7 side of native Rust modules on the Cortex-M4: boots the M4 image, serves
  its frames and supervises it (heartbeat, fault record, restart budget). The
  M7 never waits on the M4: shared-memory operations never block, ring-full is
  a counted drop, and the host thread waits only on its own doorbell
  semaphore, with a timeout.
 */
namespace RustM4 {
// thread_create() offset: the host on PRIORITY_UART (on ChibiOS 61; the bounds are checked in
// AP_RustM4_ChibiOS.cpp)
static const int8_t HOST_PRIORITY_OFFSET = 1;
}

class AP_RustM4 {
public:
    AP_RustM4();
    CLASS_NO_COPY(AP_RustM4);

    static AP_RustM4 *get_singleton() { return _singleton; }

    enum class State : uint8_t {
        Held = 0,         // not started: RUST_ENABLE was 0 at boot
        Starting = 1,
        Running = 2,
        Restarting = 3,
        Off = 4,          // until reboot
    };

    // start the M4 and the host thread; called once from AP_Rust::Runtime::init()
    void init(const AP_Rust::Runtime::Config &config);
    // from AP_Rust::Runtime::update(): a RUST_RESTART change asks for an M4 restart
    void update(const AP_Rust::Runtime::Config &config);
    bool arming_checks(size_t buflen, char *buffer) const;
    State state() const { return _state; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class ServeResult : uint8_t {
        Idle,     // the ring is empty
        Busy,     // the per-wake budget ran out with a frame just served; more may remain
        Stuck,    // nothing can be served now: not running, or a protocol error
    };

    // RUST_M4_OPTS bit 0
    static const uint8_t OPTION_RPC_SUMMARY = 1U << 0;

    AP_Int8 _restart_max;
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
    AP_Int8 _inject;
    int8_t _inject_seen;
#endif
    AP_Int8 _options;

    void host_thread();
    void pause_for_duty_cap(uint32_t pause_us, bool listening);
    ServeResult serve();
    void charge_cpu(bool pass_end);
    void handle_message(const uint8_t *buf, uint16_t len);
    void send_ready(uint32_t now_ms);
    void log_rpc_stats(const uint8_t *buf);
    // a module's text notify: sent as the PR's Runtime sends module texts
    void send_module_text(const uint8_t *buf, uint16_t len);
    void log_supervisor(uint32_t now_ms, uint32_t elapsed_ms);
    void supervise(uint32_t now_ms);
    void check_started(uint32_t now_ms);
    void check_running(uint32_t now_ms);
    void failure(const char *reason, const char *detail = nullptr);
    bool report_start_fault();
    void go_off(const char *reason);
    void grant_restart();
    void run_control(uint32_t now_ms);
    RustM4::RunAction take_run_request(uint32_t running_ms);
    void request_restart();
    void enter(State state, uint32_t now_ms);
    uint8_t restart_limit() const;
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
    void send_fault_inject();
#endif

    // per platform: AP_RustM4_ChibiOS.cpp, AP_RustM4_SITL.cpp
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
    uint32_t _restart_grants;         // restarts granted the M4: fault restarts and requested ones
    bool _restart_requested;          // the restart under way is a request, not a fault
    bool _initialized;
    bool _prearm_disabled;            // RUST_DEBUG_OPTS bit 4, as last seen by update()
    int16_t _observed_restart;        // RUST_RESTART when update() last looked
    RustM4::RunRequests _run_requests;   // RUST_RESTART changes, taken by the host thread
    RustM4::RunControl _run_control;
    uint8_t _module_count;            // modules of the current session
    uint32_t _m4_session;             // M4 peer header session being served
    uint32_t _m4_fault_count;         // M4 fault count already handled
    uint32_t _m4_fault_pc;
    uint32_t _m4_heartbeat;
    uint32_t _m4_heartbeat_ms;
    // a fault at a fault restart's start: the old session's on its way down, or the new session's
    // own before this M7 read its header; reported if the session never beats (report_start_fault()).
    // Restart requests wait while it is pending
    RustM4::StartFault _start_fault;
    uint32_t _woke_us;
    uint32_t _wake_latency_us;

    uint8_t _rx_frame[RustM4::MAX_FRAME];
    uint32_t _session_calls;          // calls of the current M4 session
    uint32_t _ready_sent_ms;

    // counters of the current one-second window
    uint32_t _stats_start_ms;
    uint32_t _requests;
    uint16_t _protocol_errors;
    uint16_t _dropped;
    uint32_t _host_busy_us;
    uint16_t _duty_cap_hits;           // pauses the duty cap forced
    uint32_t _host_cpu_us;             // host CPU time, what the duty cap charges
    uint16_t _long_passes;             // passes over LONG_PASS_US of wall time
    uint32_t _pass_cpu_max_us;         // the largest CPU time of one pass
    // the host's share of the M7: it runs above the I/O threads, a flooding M4 must not starve them
    RustM4::ServeDutyCap _duty_cap;
    // ChibiOS charges all the host's CPU (its waits sleep); SITL only its passes (its waits spin)
    RustM4::HostCpuCharge _cpu_charge { CONFIG_HAL_BOARD != HAL_BOARD_SITL };

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

    static AP_RustM4 *_singleton;
};

#endif  // AP_RUST_M4_ENABLED
