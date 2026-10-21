#pragma once

#include "AP_ModuleM4_config.h"

#if AP_MODULE_M4_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_Module/Runtime.h>
#include "AP_ModuleM4_Discovery.h"
#include "AP_ModuleM4_FileServer.h"
#include "AP_ModuleM4_Protocol.h"
#include "AP_ModuleM4_RunControl.h"
#include "AP_ModuleM4_Server.h"
#include "AP_ModuleM4_ServeWait.h"
#include "AP_ModuleM4_StartFault.h"

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
namespace ModuleM4 {
// thread_create() offsets: the host on PRIORITY_UART, the slow worker on PRIORITY_IO (on ChibiOS
// 61 and 56; the bounds are checked in AP_ModuleM4_ChibiOS.cpp)
static const int8_t HOST_PRIORITY_OFFSET = 1;
static const int8_t WORKER_PRIORITY_OFFSET = -2;
}

class AP_ModuleM4 {
public:
    AP_ModuleM4();
    CLASS_NO_COPY(AP_ModuleM4);

    static AP_ModuleM4 *get_singleton() { return _singleton; }

    enum class State : uint8_t {
        Held = 0,         // not started: MOD_ENABLE was 0 at boot
        Starting = 1,
        Running = 2,
        Restarting = 3,
        Off = 4,          // until reboot
    };

    // start the M4 and the host thread; called once from AP_Module::Runtime::init()
    void init(const AP_Module::Runtime::Config &config);
    // from AP_Module::Runtime::update(): a MOD_RESTART change asks for an M4 restart
    void update(const AP_Module::Runtime::Config &config);
    bool arming_checks(size_t buflen, char *buffer) const;
    State state() const { return _state; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class ServeResult : uint8_t {
        Idle,     // the ring is empty
        Busy,     // the per-wake budget ran out with a frame just served; more may remain
        Stuck,    // nothing can be served now: not running, or a protocol error
    };

    // MOD_M4_OPTS bit 0
    static const uint8_t OPTION_RPC_SUMMARY = 1U << 0;
    // MOD_M4_OPTS bit 1: the M4's CRC self-check values, once per session
    static const uint8_t OPTION_CRC_VALUES = 1U << 1;

    AP_Int8 _restart_max;
#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
    AP_Int8 _inject;
    int8_t _inject_seen;
#endif
    AP_Int8 _options;

    void host_thread();
    void worker_thread();
    void serve_on_worker(uint16_t len);
    bool refuse_slow_binding(const uint8_t *buf, uint16_t len);
    void serve_binding(const uint8_t *buf, uint16_t len);
    void serve_system(const uint8_t *buf, uint16_t len);
    // the session's module files, found on the slow worker before Ready (discover())
    void discover_on_worker();
    void discover();
    void load_error(const char *reason);
    // system calls, on the slow worker; each returns the reply's length in _tx_frame
    uint16_t serve_config(const uint8_t *buf, uint16_t len, uint32_t seq);
    uint16_t serve_list_modules(uint32_t seq);
    uint16_t serve_resolve(const uint8_t *buf, uint16_t len, uint32_t seq);
    void module_state(const uint8_t *buf, uint16_t len);
    void drain_module_texts();
    uint16_t serve_file(ModuleM4::SystemMethod method, const uint8_t *buf, uint16_t len, uint32_t seq);
    uint16_t pack_system_reply(ModuleM4::SystemMethod method, uint32_t seq, ModuleM4::ReplyStatus transport,
                               int32_t result, uint16_t length);
    void send_frame(const uint8_t *buf, uint16_t len);
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
    ModuleM4::RunAction take_run_request(uint32_t running_ms);
    void request_restart();
    void enter(State state, uint32_t now_ms);
    uint8_t restart_limit() const;
#if AP_MODULE_M4_FAULT_INJECTION_ENABLED
    void send_fault_inject();
#endif

    // per platform: AP_ModuleM4_ChibiOS.cpp, AP_ModuleM4_SITL.cpp
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
    bool _prearm_disabled;            // MOD_DEBUG_OPTS bit 4, as last seen by update()
    int16_t _observed_restart;        // MOD_RESTART when update() last looked
    ModuleM4::RunRequests _run_requests;   // MOD_RESTART changes, taken by the host thread
    ModuleM4::RunControl _run_control;
    uint8_t _module_count;            // modules of the current session: discovered, loadable
    ModuleM4::ModuleFile _modules[ModuleM4::MAX_MODULES];   // their files, sorted by name
    char _load_error[152];            // why this session loads no module, for the pre-arm check
    uint8_t _modules_loaded;          // ModuleState(Loaded) of this session
    bool _config_seen;                // this session's Config arrived: its CRC self-check is known
    bool _crc_failed_sent;            // texts of this session
    bool _crc_values_sent;
    bool _crc_failed;                 // an M4 CRC self-check failed: modules stay off until reboot
    bool _code_size_capped_sent;
    AP_ModuleM4_FileServer _file_server { ModuleM4::filesystem_file_ops() };
    uint32_t _m4_session;             // M4 peer header session being served
    uint32_t _m4_fault_count;         // M4 fault count already handled
    uint32_t _m4_fault_pc;
    uint32_t _m4_heartbeat;
    uint32_t _m4_heartbeat_ms;
    // a fault at a fault restart's start: the old session's on its way down, or the new session's
    // own before this M7 read its header; reported if the session never beats (report_start_fault()).
    // Restart requests wait while it is pending
    ModuleM4::StartFault _start_fault;
    uint32_t _woke_us;
    uint32_t _wake_latency_us;

    // module texts as the PR's Runtime::text queues them: 44 characters each, sent by the host thread
    struct ModuleText {
        uint8_t severity;
        char text[45];
    };
    ModuleText _texts[8];
    uint8_t _text_head;
    uint8_t _text_count;
    uint32_t _texts_dropped;
    HAL_Semaphore _text_sem;

    uint8_t _rx_frame[ModuleM4::MAX_FRAME];
    uint8_t _tx_frame[ModuleM4::MAX_FRAME];
    // the modules' binding calls; the host and the slow worker it waits for, one at a time
    AP_ModuleM4_Server _server;
    uint32_t _session_calls;          // calls of the current M4 session
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
    ModuleM4::RefusalNotice _refusal_notice;   // GCS texts about those refusals
    // the host's share of the M7: it runs above the I/O threads, a flooding M4 must not starve them
    ModuleM4::ServeDutyCap _duty_cap;
    // ChibiOS charges all the host's CPU (its waits sleep); SITL only its passes (its waits spin)
    ModuleM4::HostCpuCharge _cpu_charge { CONFIG_HAL_BOARD != HAL_BOARD_SITL };

    // the slow worker, below the I/O threads, serves the frames the fast lane does not
    // (AP_ModuleM4_Server::frame_on_fast_lane()): the host hands it one frame (_rx_frame) and waits
    // for it, so the M4 sees its replies in order and only one of the two threads serves at a time
    HAL_BinarySemaphore _worker_request;
    HAL_BinarySemaphore _worker_done;
    uint16_t _worker_len;             // 0: discover() rather than a frame
    // the worker's CPU per frame (pass only, as its waits are not the host's), read by the host
    // after _worker_done
    ModuleM4::HostCpuCharge _worker_cpu_charge { false };
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

    static AP_ModuleM4 *_singleton;
};

#endif  // AP_MODULE_M4_ENABLED
