#pragma once

#include "AP_RustM4_config.h"

#if AP_RUST_M4_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <AP_Rust/Runtime.h>
#include "AP_RustM4_Discovery.h"
#include "AP_RustM4_FileServer.h"
#include "AP_RustM4_Protocol.h"
#include "AP_RustM4_RunControl.h"
#include "AP_RustM4_Server.h"
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
// thread_create() offsets: the host on PRIORITY_UART, the slow worker on PRIORITY_IO (on ChibiOS
// 61 and 56; the bounds are checked in AP_RustM4_ChibiOS.cpp)
static const int8_t HOST_PRIORITY_OFFSET = 1;
static const int8_t WORKER_PRIORITY_OFFSET = -2;
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
    bool arming_checks(size_t buflen, char *buffer, const AP_Rust::Runtime::Config &config) const;
    // the PR's masked XORs of the whole-file CRC-32s of the loaded and of the running modules
    void checksums(uint32_t &loaded, uint32_t &running) const;
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
    // RUST_M4_OPTS bit 1: the M4's CRC self-check values, once per session
    static const uint8_t OPTION_CRC_VALUES = 1U << 1;

    AP_Int8 _restart_max;
#if AP_RUST_M4_FAULT_INJECTION_ENABLED
    AP_Int8 _inject;
    int8_t _inject_seen;
#endif
    AP_Int8 _options;

    void host_thread();
    void worker_thread();
    void serve_on_worker(uint16_t len);
    bool refuse_slow_binding(const uint8_t *buf, uint16_t len);
    void serve_binding(const uint8_t *buf, uint16_t len);
    // the AP_Param change observer: any thread, lock-free (AP_RustM4_Server::mark_changed())
    static void parameter_changed(const AP_Param *param);
    // the dirty slots' values on the slow worker, charged to the duty cap like a worker frame
    void mirror_on_worker();
    void send_mirror_changes();
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
    uint16_t serve_file(RustM4::SystemMethod method, const uint8_t *buf, uint16_t len, uint32_t seq);
    uint16_t pack_system_reply(RustM4::SystemMethod method, uint32_t seq, RustM4::ReplyStatus transport,
                               int32_t result, uint16_t length);
    // true when sent
    bool send_frame(const uint8_t *buf, uint16_t len);
    void pause_for_duty_cap(uint32_t pause_us, bool listening);
    ServeResult serve();
    void charge_cpu(bool pass_end);
    void handle_message(const uint8_t *buf, uint16_t len);
    void send_ready(uint32_t now_ms);
    void log_rpc_stats(const uint8_t *buf);
    void take_counters(const uint8_t *buf);
    // a module's text notify: sent as the PR's Runtime sends module texts
    void send_module_text(const uint8_t *buf, uint16_t len);
    void log_supervisor(uint32_t now_ms, uint32_t elapsed_ms);
    void supervise(uint32_t now_ms);
    void check_started(uint32_t now_ms);
    void check_running(uint32_t now_ms);
    // detail: the fault record's text; location: where in a module it faulted, each sent on its own line
    void failure(const char *reason, const char *detail = nullptr, const char *location = nullptr);
    // an M4 fault record's texts, and the module location of its PC
    void report_fault(uint32_t cfsr, uint32_t hfsr, uint32_t pc, uint32_t lr);
    // the session's module states and checksums start over (a session start, a restart, off)
    void clear_modules();
    bool module_ended(uint8_t index);
    bool report_start_fault();
    void go_off(const char *reason);
    void grant_restart();
    void run_control(uint32_t now_ms);
    // a stop of every running module (Control), before a requested restart
    void stop_modules();
    void check_stopped();
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
    bool _enabled;                    // RUST_ENABLE, as last seen by update()
    // a fault restart waits restart_backoff_ms() before its grant: on the timeout clock
    bool _grant_pending;
    uint32_t _grant_due_timeout_ms;
    // a requested restart waits for the running modules to stop, at most MODULE_STOP_TIMEOUT_MS
    bool _stopping;
    uint32_t _stop_start_timeout_ms;
    int16_t _observed_restart;        // RUST_RESTART when update() last looked
    RustM4::RunRequests _run_requests;   // RUST_RESTART changes, taken by the host thread
    RustM4::RunControl _run_control;
    uint8_t _module_count;            // modules of the current session: discovered, loadable
    RustM4::ModuleFile _modules[RustM4::MAX_MODULES];   // their files, sorted by name
    char _load_error[152];            // why this session loads no module, for the pre-arm check
    uint8_t _modules_loaded;          // ModuleState(Loaded) of this session
    // per module of this session: from its Loaded, its Started until Returned, Panicked or Failed
    bool _module_loaded[RustM4::MAX_MODULES];
    bool _module_running[RustM4::MAX_MODULES];
    uint32_t _module_crc[RustM4::MAX_MODULES];   // the CRC-32 the M7 served the M4 for it
    uint32_t _served_crc;             // of the last module file the M4 closed
    uint32_t _loaded_checksum;        // XOR of _module_crc of the loaded modules
    uint32_t _running_checksum;       // ... of the running ones
    // module 0's code and stack, from its Loaded, for the fault texts
    uint32_t _module_rx_base;
    uint32_t _module_rx_size;
    uint32_t _module_stack_base;
    bool _config_seen;                // this session's Config arrived: its CRC self-check is known
    bool _crc_failed_sent;            // texts of this session
    bool _crc_values_sent;
    bool _crc_failed;                 // an M4 CRC self-check failed: modules stay off until reboot
    bool _code_size_capped_sent;
    AP_RustM4_FileServer _file_server { RustM4::filesystem_file_ops() };
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

    uint8_t _rx_frame[RustM4::MAX_FRAME];
    uint8_t _tx_frame[RustM4::MAX_FRAME];
    // the modules' binding calls; the host and the slow worker it waits for, one at a time
    AP_RustM4_Server _server;
    uint32_t _session_calls;          // calls of the current M4 session
    // ParameterChanged frames: packed here, as a reply may wait in _tx_frame
    uint8_t _mirror_frame[RustM4::MAX_FRAME];
    uint32_t _mirror_backstop_ms;
    uint32_t _mirror_frames_sent;
    uint32_t _mirror_entries_sent;
    uint32_t _mirror_failed_sends;
    // posted logger.write frames: run with a status other than OK, and refused unrun (quota, duty cap)
    uint32_t _post_errors;
    uint32_t _post_drops;
    bool _post_error_reported;        // the GCS text of a failed one, once per module load
    uint32_t _posted_batched;         // posted frames the worker served after the one handed to it
    // the M4's Counters frames since the last M4MR row
    uint32_t _m4_mirror_hits;
    uint32_t _m4_mirror_misses;
    uint32_t _m4_post_drops;
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
    RustM4::RefusalNotice _refusal_notice;   // GCS texts about those refusals
    // the host's share of the M7: it runs above the I/O threads, a flooding M4 must not starve them
    RustM4::ServeDutyCap _duty_cap;
    // ChibiOS charges all the host's CPU (its waits sleep); SITL only its passes (its waits spin)
    RustM4::HostCpuCharge _cpu_charge { CONFIG_HAL_BOARD != HAL_BOARD_SITL };

    // the slow worker, below the I/O threads, serves the frames the fast lane does not
    // (AP_RustM4_Server::frame_on_fast_lane()): the host hands it one frame (_rx_frame) and waits
    // for it, so the M4 sees its replies in order and only one of the two threads serves at a time
    HAL_BinarySemaphore _worker_request;
    HAL_BinarySemaphore _worker_done;
    enum class WorkerJob : uint8_t { Frame, Discover, Mirror };
    WorkerJob _worker_job;
    uint16_t _worker_len;             // the frame's length for WorkerJob::Frame
    // after a posted batch: the length of the first other frame, left in _rx_frame for the host
    uint16_t _worker_handback_len;
    // the worker's CPU per frame (pass only, as its waits are not the host's), read by the host
    // after _worker_done
    RustM4::HostCpuCharge _worker_cpu_charge { false };
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

    static AP_RustM4 *_singleton;
};

#endif  // AP_RUST_M4_ENABLED
