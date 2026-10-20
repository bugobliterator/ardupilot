#pragma once

#include <AP_Rust/AP_Rust_config.h>

#if AP_SCRIPTING_RUST_ENABLED

#include <stdint.h>
#include "AP_RustM4_CallStatus.h"
#include "AP_RustM4_ModuleTable.h"
#include "AP_RustM4_Protocol.h"
#include "AP_RustM4_Quota.h"

// test-only inspection of the server (tests/wscript defines it for test_rust_m4_server)
#ifndef AP_RUST_M4_TEST_HOOKS
#define AP_RUST_M4_TEST_HOOKS 0
#endif

/*
  M7 side of a module's binding call: the PR's generated binding_dispatch() with the module's
  ModuleSlot as owner. The adapters do not all check their sizes, so whatever a frame says they
  read a zeroed 4096 B input copy and write a 4096 B output buffer; the frame limits are the only
  size checks. One thread at a time (the host thread or the slow worker it waits for).
 */
class AP_RustM4_Server {
public:
    // twice the frame: no adapter can reach past memory the server owns
    static const uint16_t SERVE_BUFFER_SIZE = 4096;

    // a new M4 session: ends every slot of the previous one, then module_count slots
    // (Discovery's count, at most RustM4::MAX_MODULES)
    void reset(uint32_t session, uint8_t module_count);
    // one BindingCall frame, untrusted; the reply is packed into reply (cap bytes), its length returned
    uint16_t serve_binding(const uint8_t *frame, uint16_t len, uint8_t *reply, uint16_t cap, uint32_t now_ms);
    // a module of this session ended: the PR's session cleanups for its slot
    void end_module(uint16_t index);
    // whether the host thread serves this frame itself (RustM4::frame_on_fast_lane())
    bool frame_on_fast_lane(const uint8_t *frame, uint16_t len) const;
    static uint8_t cleanup_count();
    uint8_t module_count() const { return _module_count; }
    // binding calls refused by their quota since the last call (M4ST QHit)
    uint32_t take_quota_hits() { return _quotas.take_hits(); }

#if AP_RUST_M4_TEST_HOOKS
    // dispatch id with both buffers guarded past RustM4::MAX_FRAME: false if the adapter wrote there
    bool dispatch_guarded(uint32_t id, uint16_t in_size, uint16_t out_size);
    void fill_input_for_test(uint8_t byte);
    // whether _in[from..SERVE_BUFFER_SIZE) was zero when the last adapter ran
    bool last_input_tail_zero(uint16_t from) const { return _last_input_end <= from; }
    bool last_dispatch_ran() const { return _last_dispatch_ran; }
    uint32_t ended_modules() const { return _ended_modules; }
    // binding_crsf_cleanup() creates and initialises AP_CRSF_Telem when disarmed, which a gtest
    // without a vehicle cannot: such a test counts end_module() calls without running the cleanups
    void run_cleanups_for_test(bool run) { _cleanups_run = run; }
    // adapters that dereference a vehicle singleton the gtest does not have
    static bool needs_vehicle(uint32_t id);
#endif

private:
    int32_t dispatch(uint16_t module, uint32_t id, uint16_t in_size, uint16_t out_size);

    uint8_t _in[SERVE_BUFFER_SIZE];
    uint8_t _out[SERVE_BUFFER_SIZE];
    RustM4::ModuleSlot _slots[RustM4::MAX_MODULES];
    uint8_t _module_count = 0;
    RustM4::Quota _quotas;
#if AP_RUST_M4_TEST_HOOKS
    uint16_t _last_input_end = 0;   // one past the last non-zero input byte at the last dispatch
    bool _last_dispatch_ran = false;
    uint32_t _ended_modules = 0;
    bool _cleanups_run = true;
#endif
};

static_assert(AP_RustM4_Server::SERVE_BUFFER_SIZE >= 2U * RustM4::MAX_FRAME, "serve buffers are twice the frame");

#endif  // AP_SCRIPTING_RUST_ENABLED
