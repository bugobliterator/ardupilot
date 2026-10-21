#pragma once

/*
  Privileged side of the M4 module runtime, independent of the platform
  (ChibiOS image or SITL process): drains the M7's messages, forwards the
  module thread's call (posted on a ModuleLink) once the M7 is Ready, times
  the calls per BindingId and reports them once per second, carries out
  fault injection and beats the M4 heartbeat. drain_step() and
  housekeeping_step() never block. The module thread sends its own call with
  forward_posted() when it can; the platform's lock serialises that with the
  service's steps, the ring's other producer.
 */

#include <stdint.h>
#include <CubePilotFW/Channel.h>
#include <AP_RustM4_Crc32.h>
#include <AP_RustM4_Protocol.h>
#include <AP_RustM4_StatsRows.h>
#include <AP_RustM4_BindingHistogram.h>
#include <AP_RustM4_ModuleTable.h>
#include <AP_RustM4_ParamMirror.h>
#include <AP_RustM4_CallProbe.h>
#include "module_link.h"

namespace RustM4 {

struct M4Platform {
    uint32_t (*millis)();
    uint32_t (*cycles)();                   // free-running counter for round trips
    uint32_t cycles_per_us;
    void (*inject_fault)(FaultKind kind);   // HardFault, MemManage, Stall, Lockup, HandlerFault
    void (*wake_module)();                  // the call on the ModuleLink is Done; may be null
    void (*start_selftest)();               // RUST_M4_INJECT 17: the built-in thread-path check; may be null
    // a Config reply's M7 millis(), just received: calibrates the M7 clock time_us follows; may be null
    void (*m7_millis)(uint32_t m7_millis);
    // one mutex around the service state and the TX ring (priority inheritance on the M4)
    void (*lock)();
    void (*unlock)();
    // a Control(CONTROL_STOP) of this session for that module; may be null
    void (*stop_module)(uint16_t module);
    // the call latency probe the module port stamps; may be null
    M4CallProbe *probe;
};

class M4Service {
public:
    void init(CubePilotFW::Channel &channel, uint8_t *window, ModuleLink &link, const M4Platform &platform);
    // after each wake: the M7's frames (replies complete the module's call) and a call the module
    // thread posted but could not send. Waits only for the lock
    void drain_step();
    // the rest, after a wait that timed out or at most once a millisecond: call timeouts, the
    // Config, fault injection's flood, the Stats rows and the heartbeat. Waits only for the lock
    void housekeeping_step();
    // the module thread, after posting: sends the call itself when the M7 is Ready and the ring
    // has room. False if it is still Posted: the service's next drain_step() sends it
    bool forward_posted();
    // the module thread, woken while it waits for its call's reply: drain_step() with that wait,
    // not the service's last one, in the probe's stamps (isr_to_thread_cycles: the doorbell interrupt
    // to this thread's return from its wait, 0 if not taken). It stops at the frame that completes
    // the call; the caller's prepare_module_sleep() then sees any frame left. A fault injection
    // drained here is left to the service's next drain_step(): true if the service must be woken for it
    bool drain_step_for_module(bool poll_timeout, bool stamped, uint32_t isr_us, uint32_t wake_us,
                               uint16_t isr_to_thread_cycles = 0);
    // the module thread, before it sleeps for its reply while the service sleeps: the Channel's
    // prepareSleep(), so the M7 rings again (once per sleep). False if a frame is waiting
    bool prepare_module_sleep() { return _channel->prepareSleep(); }
    // the module thread, as it starts polling for its reply: the M4 reads as awake, so the M7's
    // send() rings no doorbell. prepare_module_sleep() arms it again
    void module_awake() { _channel->wake(); }
    // a frame from the M7 is on the ring (its indices only, nothing read)
    bool frame_waiting() const;
    // the platform CRC's results of this session, computed at init(), which faults (HardFault)
    // when they are not zlib's
    const CrcSelfCheck &crc_self_check() const { return _crc_self_check; }
    // this session's Config (the service sends it after Ready, with the CRC self-check): its reply
    // is valid once config_ready()
    bool config_ready() const { return _config_ready; }
    const ConfigReply &config_reply() const { return _config_reply; }
    // the Stats name of a remote import of this session (cleared at init()); later rows are id:<n>
    void set_binding_name(uint32_t id, const char *name);
    // the platform's wait before this step: true if it ended on its poll timeout, not a doorbell
    // (the call latency probe flags a reply seen after one). stamped: the wait ended on a doorbell
    // interrupt the probe stamped at isr_us, and the thread resumed at wake_us
    void note_wake(bool poll_timeout, bool stamped = false, uint32_t isr_us = 0, uint32_t wake_us = 0)
    {
        _service_wake = WakeNote { poll_timeout, stamped, isr_us, wake_us, 0 };
    }
    // sent calls that had no reply within CALL_TIMEOUT_MS or lost their session
    uint32_t lost() const { return _lost; }
    // frames from the M7 dropped as not for this M4 (wrong type, stale seq or session)
    uint32_t protocol_errors() const { return _protocol_errors; }
    // the module thread, inside its binding call; each takes the service lock
    bool mirror_get(uint16_t module, uint64_t token, uint8_t *out5);
    void mirror_pending(uint16_t module, uint64_t token);
    void mirror_settle(uint16_t module, uint64_t token);
    void mirror_refresh(uint16_t module, uint64_t token, const uint8_t *out5);
    uint32_t mirror_hits() const { return _mirror_hits; }
    uint32_t mirror_misses() const { return _mirror_misses; }
    // posted binding frames that timed out before reaching the ring; their module already got OK
    uint32_t post_drops() const { return _post_drops; }

    // Stats keys: a BindingId below SYSTEM_KEY; a system method is SYSTEM_KEY | method
    static const uint16_t SYSTEM_KEY = 0x8000;
    // the folded rows, and the key of an id too large for a row of its own
    static const uint16_t KEY_OTHER = 0x7FFF;
    static const uint16_t KEY_SYSTEM_OTHER = 0xFFFF;

private:
    // a wait before a drain: ended on the poll timeout, or on a doorbell the probe stamped
    struct WakeNote {
        bool poll_timeout;
        bool stamped;
        uint32_t isr_us;
        uint32_t wake_us;
        uint16_t isr_to_thread_cycles;
    };
    // a reply's round trip, kept until fold_round_trip() adds it to its Stats row
    struct RoundTrip {
        bool pending;
        uint16_t key;
        uint32_t cycles;
        uint32_t m7_us;
    };
    struct MethodStats {
        bool used;
        bool posted;   // in the padding after used
        uint16_t key;
        BindingHistogram round_trips;
        uint32_t total_us;
        uint32_t m7_max_us;
        uint32_t lost;
    };
    // round trips are timed per key in up to STATS_SLOTS slots per window; the STATS_REPORTED
    // with the most round-trip time get a Stats row each, the rest fold into m4:other and
    // m4:sysother
    static const uint8_t STATS_SLOTS = 32;
    // names of this session's remote imports: 64 rows of 32 B in privileged RAM
    struct BindingName {
        uint32_t id;
        char name[32];
    };
    static const uint8_t BINDING_NAMES = 64;

    static StatsRow row_of(const MethodStats &slot);
    static void clear(MethodStats &slot);
    // one Stats frame, named from the key; the name is logged by the M7, never trusted
    bool send_row(const StatsRow &row);
    void row_name(uint16_t key, char *name, uint32_t size) const;
    MethodStats *stats_for(uint16_t key);
    void send_stats(uint32_t now_ms);
    void handle_ready(const MsgHeader &header);
    void handle_reply(const uint8_t *buf, uint16_t len, const MsgHeader &header);
    void handle_parameter_changed(const uint8_t *buf, uint16_t len, const MsgHeader &header);
    void forget_ended_module();
    void send_config(uint32_t now_ms);
    void handle_config_reply(const uint8_t *buf, uint16_t len);
    void count_lost();
    void record_posted(uint16_t key, uint32_t post_us);
    // the last reply's round trip into its Stats row: before any other Stats change, and after the
    // next call is on the ring, off the reply path
    void fold_round_trip();
    bool posted_binding_on_link() const;
    // the M7 announced it sleeps: a send() now rings its doorbell (the probe's FwdBell)
    bool peer_asleep() const;
    // the probe's cycle counter, when the probe is on; 0 otherwise
    uint32_t probe_cycles() const;
    // the service lock, its take timed into _lock_cycles while the probe is on
    void lock_timed();

    void drain();
    // the call timeout runs from posting: waiting for Ready or ring space counts too
    void time_posted_call();
    void handle(const uint8_t *buf, uint16_t len);
    void handle_fault_inject(const uint8_t *buf);
    void handle_control(const uint8_t *buf, const MsgHeader &header);
    // from_step: the service's step sends it, not the module thread
    void forward_call(bool from_step);
    void complete(int32_t result);
    void corrupt_tx_ring();
    void flood();
    void compute_crc_self_check();

    CubePilotFW::Channel *_channel;
    uint8_t *_window;
    ModuleLink *_link;
    M4Platform _platform;
    MethodStats _stats[STATS_SLOTS];
    StatsRow _untimed;          // calls of this window that found no free slot (the m4:other share)
    RoundTrip _round_trip;
    BindingName _names[BINDING_NAMES];
    uint8_t _name_count;
    uint32_t _seq;              // one counter per session: calls and Stats rows
    uint32_t _session;          // the seq of the M7's Ready: its session
    uint32_t _call_seq;
    uint32_t _call_session;
    uint16_t _call_key;
    uint8_t _call_reply_type;   // MsgType the Sent call is answered with
    uint32_t _call_sent_cycles;
    uint32_t _call_posted_cycles;   // when _call_posted_ms was taken: a posted binding's round trip
    uint32_t _post_drops;
    uint32_t _last_stats_ms;
    uint32_t _lost;
    uint32_t _call_posted_ms;   // when the service first saw the call Posted
    uint32_t _protocol_errors;  // frames from the M7 that are not a message the M4 serves
    uint32_t _flood_until_ms;
    bool _ready;
    bool _have_session;
    bool _call_timed;           // _call_posted_ms belongs to the current call
    bool _rx_stalled;
    bool _flooding;
    bool _call_probed;          // the Sent call carries a probe trailer
    // the probe's cycle parts of the last lock take and ring read, for the call they belong to
    bool _lock_timed;
    uint16_t _lock_cycles;
    uint16_t _receive_cycles;
    uint32_t _received_cycles;  // the counter as that read returned
    // written by note_wake() on the service thread, outside the lock: the module thread's drain
    // keeps its own
    WakeNote _service_wake;
    WakeNote _module_wake;
    bool _module_draining;      // under the lock: the module thread drains
    uint8_t _deferred_fault;    // FaultKind drained by the module thread, injected by the service
    CrcSelfCheck _crc_self_check;
    ConfigReply _config_reply;
    uint32_t _config_seq;       // of the Config sent this session; 0: none sent yet
    uint32_t _config_sent_ms;
    bool _config_pending;       // this session's Config is not answered yet
    bool _config_ready;
    ParamMirror _mirror;
    uint32_t _mirror_hits;
    uint32_t _mirror_misses;
    uint32_t _reported_mirror_hits;     // the counters' values in the last Counters frame sent
    uint32_t _reported_mirror_misses;
    uint32_t _reported_post_drops;
    uint8_t _ended_modules;     // a bit per module index: Returned, Panicked or Failed this session
    static_assert(MAX_MODULES <= 8, "_ended_modules");
    alignas(4) uint8_t _rx[MAX_FRAME];   // the ring copies word-wise into a 4-aligned buffer
};

}  // namespace RustM4
