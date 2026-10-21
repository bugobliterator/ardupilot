#pragma once

/*
  Privileged side of the M4 module runtime, independent of the platform
  (ChibiOS image or SITL process): drains the M7's messages, forwards the
  module thread's call (posted on a ModuleLink) once the M7 is Ready, times
  the calls per BindingId and reports them once per second, carries out
  fault injection and beats the M4 heartbeat. step() never blocks. The module thread sends its own call with forward_posted() when it
  can; the platform's lock serialises that with step(), the ring's other
  producer.
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
    // one pass; waits only for the lock
    void step();
    // the module thread, after posting: sends the call itself when the M7 is Ready and the ring
    // has room. False if it is still Posted: the service's next step sends it
    bool forward_posted();
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
    // (the call latency probe flags a reply seen after one)
    void note_wake(bool poll_timeout) { _woke_on_poll = poll_timeout; }
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
    bool posted_binding_on_link() const;

    void step_locked();
    void drain();
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
    uint32_t _call_posted_ms;   // when step() first saw the call Posted
    uint32_t _protocol_errors;  // frames from the M7 that are not a message the M4 serves
    uint32_t _flood_until_ms;
    bool _ready;
    bool _have_session;
    bool _call_timed;           // _call_posted_ms belongs to the current call
    bool _rx_stalled;
    bool _flooding;
    bool _call_probed;          // the Sent call carries a probe trailer
    bool _woke_on_poll;
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
    uint8_t _rx[MAX_FRAME];
};

}  // namespace RustM4
