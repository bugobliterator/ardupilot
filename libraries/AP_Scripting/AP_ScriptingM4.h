#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

namespace CubePilotFW {
class Channel;
}

/*
  M7 side of Lua scripting on the Cortex-M4: boots the M4 image, serves its
  requests and supervises it (heartbeat, fault record, restart budget). The M7
  never waits on the M4: shared-memory operations never block, ring-full is a
  counted drop, and the host thread waits only on its own doorbell semaphore,
  with a timeout.
 */
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
    bool arming_checks(size_t buflen, char *buffer) const;
    void set_summary_enabled(bool enabled) { _summary_enabled = enabled; }
    State state() const { return _state; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _restart_max;
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    AP_Int8 _inject;
    int8_t _inject_seen;
#endif

    void host_thread();
    bool serve();
    void handle_message(const uint8_t *buf, uint16_t len);
    void log_rpc_stats(const uint8_t *buf);
    void log_supervisor(uint32_t now_ms, uint32_t elapsed_ms);
    void supervise(uint32_t now_ms);
    void check_started(uint32_t now_ms);
    void check_running(uint32_t now_ms);
    void failure(const char *reason);
    void go_off(const char *reason);
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

    static void doorbell_isr(uint8_t group);

    CubePilotFW::Channel *_channel;
    uint8_t *_shared;                 // the shared window
    HAL_BinarySemaphore _doorbell;
    volatile uint32_t _doorbell_us;   // when the last doorbell interrupt arrived, 0 once consumed
    State _state;
    uint32_t _state_start_ms;
    uint8_t _restarts_used;
    uint32_t _m4_session;             // M4 peer header session being served
    uint32_t _m4_fault_count;         // M4 fault count already handled
    uint32_t _m4_fault_pc;
    uint32_t _m4_heartbeat;
    uint32_t _m4_heartbeat_ms;
    uint32_t _woke_us;
    uint32_t _wake_latency_us;

    // counters of the current one-second window
    uint32_t _stats_start_ms;
    uint32_t _requests;
    uint16_t _protocol_errors;
    uint16_t _dropped;
    uint32_t _host_busy_us;
    bool _summary_enabled;

    struct {
        uint32_t count;
        uint32_t p50_us;
        uint32_t p99_us;
        uint32_t max_us;
        uint32_t window_ms;     // time the report covers; longer than 1 s after a failed send on the M4
    } _m4_round_trips;          // last Stats report from the M4
    uint32_t _m4_stats_ms;      // when it arrived
    uint32_t _m4_pings_lost;    // lost pings reported since the last M4ST row
    uint32_t _last_summary_ms;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    int _m4_pid;
    bool spawn_m4(char *reason, size_t reason_len);
#endif

    static AP_ScriptingM4 *_singleton;
};

#endif  // AP_SCRIPTING_M4_ENABLED
