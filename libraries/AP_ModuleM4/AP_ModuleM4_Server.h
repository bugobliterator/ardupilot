#pragma once

#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED

#include <stdint.h>
#include <AP_Scripting/Parameter.h>
#include "AP_ModuleM4_CallStatus.h"
#include "AP_ModuleM4_ModuleTable.h"
#include "AP_ModuleM4_Protocol.h"
#include "AP_ModuleM4_Quota.h"

// test-only inspection of the server (tests/wscript defines it for test_module_m4_server)
#ifndef AP_MODULE_M4_TEST_HOOKS
#define AP_MODULE_M4_TEST_HOOKS 0
#endif

/*
  M7 side of a module's binding call: the PR's generated binding_dispatch() with the module's
  ModuleSlot as owner. The adapters do not all check their sizes, so whatever a frame says they
  read a zeroed 4096 B input copy and write a 4096 B output buffer; the frame limits are the only
  size checks. One thread at a time (the host thread or the slow worker it waits for).
 */
class AP_ModuleM4_Server {
public:
    AP_ModuleM4_Server();

    // twice the frame: no adapter can reach past memory the server owns
    static const uint16_t SERVE_BUFFER_SIZE = 4096;
    // Parameter handles of all modules (open addressing, a power of two) and the most in use
    static const uint16_t PARAMETER_HANDLE_SLOTS = 256;
    static const uint16_t PARAMETER_HANDLES_MAX = 224;

    // a new M4 session: ends every slot of the previous one, then module_count slots
    // (Discovery's count, at most ModuleM4::MAX_MODULES)
    void reset(uint32_t session, uint8_t module_count);
    // one BindingCall frame, untrusted; the reply is packed into reply (cap bytes), its length returned
    uint16_t serve_binding(const uint8_t *frame, uint16_t len, uint8_t *reply, uint16_t cap, uint32_t now_ms);
    // a module of this session ended: the PR's session cleanups for its slot
    void end_module(uint16_t index);
    // whether the host thread serves this frame itself (ModuleM4::frame_on_fast_lane())
    bool frame_on_fast_lane(const uint8_t *frame, uint16_t len) const;
    static uint8_t cleanup_count();
    uint8_t module_count() const { return _module_count; }
    // binding calls refused by their quota since the last call (M4ST QHit)
    uint32_t take_quota_hits() { return _quotas.take_hits(); }

#if AP_MODULE_M4_TEST_HOOKS
    // dispatch id with both buffers guarded past ModuleM4::MAX_FRAME: false if the adapter wrote there
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
    uint16_t parameter_handles() const { return _handle_count; }
    // AP_Param::find() needs the vehicle's parameter tables, which a gtest does not have: init then
    // claims the handle and leaves it unbound (as for a missing name)
    void bind_parameters_for_test(bool bind) { _bind_parameters = bind; }
#endif

private:
    int32_t dispatch(uint16_t module, uint32_t id, uint16_t in_size, uint16_t out_size);
    // the PR's Parameter.* adapters take the address of a Parameter in module memory and use
    // it: here the address is a token only, of an M7-owned Parameter of the module
    int32_t serve_parameter(uint16_t module, uint32_t id);
    int16_t find_handle(uint16_t module, uint64_t token, bool insert);
    void erase_handle(uint16_t slot);
    void release_handles(uint16_t module);
    void clear_handles();
    static uint16_t handle_home(uint16_t module, uint64_t token);

    uint8_t _in[SERVE_BUFFER_SIZE];
    uint8_t _out[SERVE_BUFFER_SIZE];
    ModuleM4::ModuleSlot _slots[ModuleM4::MAX_MODULES];
    uint8_t _module_count = 0;
    ModuleM4::Quota _quotas;
    static const uint8_t HANDLE_FREE = 0xFF;
    uint64_t _handle_token[PARAMETER_HANDLE_SLOTS];
    Parameter _handle[PARAMETER_HANDLE_SLOTS];
    uint8_t _handle_module[PARAMETER_HANDLE_SLOTS];
    uint16_t _handle_count = 0;
#if AP_MODULE_M4_TEST_HOOKS
    uint16_t _last_input_end = 0;   // one past the last non-zero input byte at the last dispatch
    bool _last_dispatch_ran = false;
    uint32_t _ended_modules = 0;
    bool _cleanups_run = true;
    bool _bind_parameters = true;
#endif
};

static_assert(AP_ModuleM4_Server::SERVE_BUFFER_SIZE >= 2U * ModuleM4::MAX_FRAME, "serve buffers are twice the frame");
static_assert((AP_ModuleM4_Server::PARAMETER_HANDLE_SLOTS & (AP_ModuleM4_Server::PARAMETER_HANDLE_SLOTS - 1)) == 0 &&
              AP_ModuleM4_Server::PARAMETER_HANDLES_MAX < AP_ModuleM4_Server::PARAMETER_HANDLE_SLOTS,
              "handle table: a power of two, never full");
static_assert(ModuleM4::MAX_MODULES < 0xFF, "a module index is never the free marker");

#endif  // AP_MODULE_ENABLED
