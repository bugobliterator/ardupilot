#pragma once

/*
  Privileged side of the M4 scripting core, independent of the platform
  (ChibiOS image or SITL process): drains the M7's messages, forwards the
  Lua thread's calls (posted on a LuaLink) once the M7 is Ready, times them
  per method, reports the times once per second, carries out fault
  injection and beats the M4 heartbeat. step() never blocks.
 */

#include <atomic>
#include <stdint.h>
#include <CubePilotFW/Channel.h>
#include <AP_Scripting_BindingHistogram.h>
#include <AP_ScriptingM4_Protocol.h>
#include <AP_ScriptingM4_StatsRows.h>

namespace ScriptingM4 {

// one Lua call between the Lua thread and the service; the request is a whole Call
// or Notify frame (its seq is set by the service), the reply a whole Reply frame
struct LuaLink {
    enum State : uint8_t {
        Idle = 0,     // the Lua thread may post
        Posted = 1,   // request filled in by the Lua side
        Sent = 2,     // on the ring, waiting for the Reply
        Done = 3,     // result (and reply) filled in by the service
    };
    std::atomic<uint8_t> state;
    std::atomic<uint8_t> fault_request;   // FaultKind for the Lua thread, None once taken
    uint16_t request_len;
    int32_t result;                       // reply length, 0 for a Notify, or -errno
    uint8_t request[MAX_FRAME];
    uint8_t reply[MAX_FRAME];
};

struct M4Platform {
    uint32_t (*millis)();
    uint32_t (*cycles)();                   // free-running counter for round trips
    uint32_t cycles_per_us;
    void (*inject_fault)(FaultKind kind);   // HardFault, MemManage, Stall, Lockup
    void (*wake_lua)();                     // the call on the LuaLink is Done
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    // this session's NET_*, once; magic 0: networking off on the M7 side. nullptr: no network
    void (*network_config)(const NetM4::Config &config);
#endif
};

class M4Service {
public:
    void init(CubePilotFW::Channel &channel, uint8_t *window, LuaLink &link, const M4Platform &platform);
    // one pass; never blocks
    void step();

private:
    struct MethodStats {
        uint16_t method;          // 0: free slot
        BindingHistogram round_trips;
        uint32_t total_us;
        uint32_t m7_max_us;
        uint32_t lost;
    };
    // round trips are timed per method in up to STATS_SLOTS slots per window; the
    // STATS_REPORTED with the most round-trip time get a Stats row each, the rest one
    // m4:other row (spec §11: SRPC, top methods by total time)
    static const uint8_t STATS_SLOTS = 32;
    static const uint8_t STATS_REPORTED = 8;
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    static const uint32_t NET_CONFIG_RETRY_MS = 1000;
#endif

    static StatsRow row_of(const MethodStats &slot);
    static void clear(MethodStats &slot);
    bool send_row(const StatsRow &row);

    void drain();
    void handle(const uint8_t *buf, uint16_t len);
    void handle_reply(const uint8_t *buf, uint16_t len, const MsgHeader &header);
    void handle_fault_inject(const uint8_t *buf);
    void forward_call();
    void complete(int32_t result);
    MethodStats *stats_for(uint16_t method);
    void send_stats(uint32_t now_ms);
    void flood();
    void corrupt_tx_ring();
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    void request_net_config(uint32_t now_ms);
#endif

    CubePilotFW::Channel *_channel;
    uint8_t *_window;
    LuaLink *_link;
    M4Platform _platform;
    MethodStats _stats[STATS_SLOTS];
    uint32_t _seq;
    uint32_t _call_seq;
    uint16_t _call_method;
    uint32_t _call_sent_cycles;
    uint32_t _call_posted_ms;   // when step() first saw the call Posted
    uint32_t _last_stats_ms;
    StatsRow _untimed;        // calls of this window that found no free slot (the m4:other share)
    uint32_t _flood_until_ms;
    bool _flooding;
    bool _ready;
    bool _call_timed;            // _call_posted_ms belongs to the current call
    bool _rx_stalled;
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    bool _net_configured;
    uint32_t _net_request_ms;
#endif
    uint8_t _rx[MAX_FRAME];
};

}  // namespace ScriptingM4
