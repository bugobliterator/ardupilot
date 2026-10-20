#pragma once

/*
  Privileged side of the M4 module runtime, independent of the platform
  (ChibiOS image or SITL process): drains the M7's messages, forwards the
  module thread's call (posted on a ModuleLink) once the M7 is Ready,
  carries out fault injection and beats the M4 heartbeat. step() never
  blocks. The module thread sends its own call with forward_posted() when it
  can; the platform's lock serialises that with step(), the ring's other
  producer.
 */

#include <stdint.h>
#include <CubePilotFW/Channel.h>
#include <AP_ModuleM4_Crc32.h>
#include <AP_ModuleM4_Protocol.h>
#include <AP_ModuleM4_StatsRows.h>
#include "module_link.h"

namespace ModuleM4 {

struct M4Platform {
    uint32_t (*millis)();
    uint32_t (*cycles)();                   // free-running counter for round trips
    uint32_t cycles_per_us;
    void (*inject_fault)(FaultKind kind);   // HardFault, MemManage, Stall, Lockup, HandlerFault
    void (*wake_module)();                  // the call on the ModuleLink is Done; may be null
    // one mutex around the service state and the TX ring (priority inheritance on the M4)
    void (*lock)();
    void (*unlock)();
};

class M4Service {
public:
    void init(CubePilotFW::Channel &channel, uint8_t *window, ModuleLink &link, const M4Platform &platform);
    // one pass; waits only for the lock
    void step();
    // the module thread, after posting: sends the call itself when the M7 is Ready and the ring
    // has room. False if it is still Posted: the service's next step sends it
    bool forward_posted();
    // the platform CRC's results of this session, computed at init()
    const CrcSelfCheck &crc_self_check() const { return _crc_self_check; }

private:
    // one Stats frame; name is logged by the M7, never trusted
    bool send_row(const StatsRow &row, const char *name);

    void step_locked();
    void drain();
    void handle(const uint8_t *buf, uint16_t len);
    void handle_fault_inject(const uint8_t *buf);
    void forward_call();
    void complete(int32_t result);
    void corrupt_tx_ring();
    void compute_crc_self_check();

    CubePilotFW::Channel *_channel;
    uint8_t *_window;
    ModuleLink *_link;
    M4Platform _platform;
    uint32_t _seq;
    uint32_t _call_seq;
    uint32_t _call_posted_ms;   // when step() first saw the call Posted
    uint32_t _protocol_errors;  // frames from the M7 that are not a message the M4 serves
    bool _ready;
    bool _call_timed;           // _call_posted_ms belongs to the current call
    bool _rx_stalled;
    CrcSelfCheck _crc_self_check;
    uint8_t _rx[MAX_FRAME];
};

}  // namespace ModuleM4
