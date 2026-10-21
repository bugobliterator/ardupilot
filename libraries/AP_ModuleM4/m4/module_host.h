#pragma once

/*
  The M4's module loader, driven from the service thread between the
  service's steps (never inside an SVC). It loads the session's module end
  to end: the cached Config, ListModules, the file staged at the bottom of
  the module area by FsOpen/FsRead/FsClose, the PR's Image::parse, one
  Resolve round, placement by load_layout(), Image::load_split, the image
  checksum against the CRC of the bytes the M7 served, then the stack, the
  heap and the start of the module thread. Its system calls go through the
  ModuleLink, which no module thread uses before the start. Every failure
  is final for the session and reported to the M7 as ModuleState(Failed)
  with the reason in the PR's words.
 */

#include <stdint.h>
#include <AP_Module/Image.h>
#include <AP_ModuleM4_HostCalls.h>
#include <AP_ModuleM4_LoadLayout.h>
#include <AP_ModuleM4_Protocol.h>
#include "module_link.h"

namespace ModuleM4 {

// the largest MOD_HEAP_MAX the loader keeps allocator bitmaps for (privileged RAM: 2 bits per
// 16 B granule); a larger heap fails the load as insufficient M4 RAM
static const uint32_t MODULE_HEAP_MAX = 64U * 1024U;

struct ModuleHostPlatform {
    ModuleLink *link;           // the host's system calls, while no module thread runs
    // the module area as the host writes it, and its address as the module sees it (the same on
    // the M4 and in the SITL M4 process; the gtests map a buffer at the M4's address)
    uint8_t *area;
    uint32_t area_base;
    uint32_t area_size;
    // this platform's parse identity; stack_limit is MOD_STACK_MAX less stack_margin
    AP_Module::Image::Compatibility compatibility;
    uint32_t stack_margin;
    void *context;
    // this session's Config reply, once the service has it
    bool (*config)(void *context, ConfigReply &reply);
    // the Stats name of a remote import
    void (*set_binding_name)(void *context, uint32_t id, const char *name);
    // the code and guard regions off before the area is written; may be null
    void (*unpublish)(void *context);
    // makes the module's code executable where its start does not; may be null
    bool (*publish)(void *context, uint8_t *code, uint32_t size);
    // module_port_start()
    void (*start)(void *context, ModuleContext &module, const LoadLayout &layout, uint32_t entry,
                  uint32_t rw_base);
    // module_port_request_stop(); may be null
    void (*request_stop)(void *context, ModuleContext &module);
};

class ModuleHost {
public:
    enum class Step : uint8_t {
        Config, List, Open, Read, Parse, Resolve, Place, Load, Checksum, Publish, Start, Running, Failed,
        Idle,   // the session has no module
    };

    void init(const ModuleHostPlatform &platform);
    // one step of the load; waits for nothing
    void step();
    Step state() const { return _step; }
    // the reason of a Failed load, as the M7 prints it after "Module: "
    const char *failure() const { return _failure; }
    ModuleContext &module() { return _module; }
    // the M7 asked the module to stop: at once if its thread runs, else as soon as it starts (the
    // start clears the module's stop flag)
    void request_stop();
    const LoadLayout &layout() const { return _layout; }

private:
    uint8_t *pointer(uint32_t address) const { return _platform.area + (address - _platform.area_base); }
    void post(uint16_t length);
    bool completed(int32_t &result);
    bool system_reply(int32_t result, SystemMethod method, uint16_t minimum, SystemReplyHeader &reply) const;
    void post_system_call(SystemMethod method, uint16_t length);
    void post_read();
    void post_module_state(ModuleStateKind kind);
    void fail(const char *detail);
    void fail_load(const char *detail);
    void list_modules(int32_t result);
    void read(int32_t result);
    void parse();
    void resolve(int32_t result);
    void place();
    void load();
    void publish();
    void start();

    ModuleHostPlatform _platform;
    Step _step;
    bool _calling;            // a posted call has not completed
    uint8_t _start_phase;     // Start: Loaded sent, Started sent
    bool _stop_pending;       // a stop asked before the module thread ran
    ConfigReply _config;
    char _name[32];           // the module's file name, as the PR's texts print it (%.31s)
    char _path[96];
    uint32_t _size;           // as listed
    int32_t _fd;
    uint32_t _length;         // staged bytes
    bool _closing;
    uint32_t _served_crc;     // FsClose: the CRC and count of the bytes the M7 served
    uint32_t _served;
    uint16_t _import_count;
    AP_Module::Image _image;
    LoadLayout _layout;
    uint32_t _rx_base, _rx_size, _rw_base;
    ModuleContext _module;
    uint32_t _heap_used[MODULE_HEAP_MAX / GranuleHeap::GRANULE / 32U];
    uint32_t _heap_starts[MODULE_HEAP_MAX / GranuleHeap::GRANULE / 32U];
    char _failure[144];
};

}  // namespace ModuleM4
