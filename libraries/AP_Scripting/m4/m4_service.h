#pragma once

/*
  M4 side of Lua scripting, independent of the platform (ChibiOS image or
  SITL process): serves the M7's messages, times Ping round trips, reports
  them once per second and beats the M4 heartbeat. Lua arrives with the
  Remote Bindings phase.
 */

#include <stdint.h>
#include <CubePilotFW/Channel.h>
#include <AP_Scripting_BindingHistogram.h>
#include <AP_ScriptingM4_Protocol.h>

namespace ScriptingM4 {

struct M4Platform {
    uint32_t (*millis)();
    uint32_t (*cycles)();                   // free-running counter for round trips
    uint32_t cycles_per_us;
    void (*inject_fault)(FaultKind kind);   // HardFault, MemManage, Stall, Lockup
};

class M4Service {
public:
    void init(CubePilotFW::Channel &channel, uint8_t *window, const M4Platform &platform);
    // one pass; never blocks
    void step();

private:
    void drain();
    void handle(const uint8_t *buf, uint16_t len);
    void send_ping(uint32_t now_ms);
    void send_stats(uint32_t now_ms);
    void corrupt_tx_ring();

    CubePilotFW::Channel *_channel;
    uint8_t *_window;
    M4Platform _platform;
    BindingHistogram _round_trips;
    uint32_t _round_trip_total_us;
    uint32_t _m7_max_us;
    uint32_t _lost;
    uint32_t _seq;
    uint32_t _ping_seq;
    uint32_t _ping_sent_cycles;
    uint32_t _ping_sent_ms;
    uint32_t _last_ping_ms;
    uint32_t _last_stats_ms;
    bool _ping_outstanding;
    bool _rx_stalled;
};

}  // namespace ScriptingM4
