/*
  SITL side of the module thread: a pthread of the M4 process calling the
  same AP_ModuleM4_HostCalls.h bodies as the M4 image's SVC handlers, with the
  same context token and local exact-size rule. There is no privilege split
  on a host: module faults are signals, recorded by main.cpp's handler (the
  thread gets its own alternate signal stack). Module code is mapped as the
  PR's ExecutableMemoryPosix does: RW to copy, then the 4096-aligned code
  prefix PROT_READ | PROT_EXEC.
 */
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <AP_Module/Bindings.h>
#include "../module_port.h"

using namespace ModuleM4;

static ModuleLink *call_link_of;
static M4Service *service_of;
static pthread_mutex_t port_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t port_cond = PTHREAD_COND_INITIALIZER;   // a call Done, a stop request
static ModuleContext *active;
static pthread_t module_thread;
static bool module_running;
static bool module_has_returned;
static int32_t module_result;
static uint32_t module_entry;
static uint8_t alternate_stack[64 * 1024];

static uint64_t monotonic_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000U + uint64_t(ts.tv_nsec) / 1000U;
}

// waits on port_cond until deadline_us (CLOCK_REALTIME based, as pthread_cond_timedwait)
static void wait_until(const struct timespec &deadline)
{
    pthread_mutex_lock(&port_mutex);
    pthread_cond_timedwait(&port_cond, &port_mutex, &deadline);
    pthread_mutex_unlock(&port_mutex);
}

static struct timespec realtime_after_ms(uint32_t ms)
{
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += ms / 1000U;
    deadline.tv_nsec += long(ms % 1000U) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    return deadline;
}

static int32_t port_call(void *, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap)
{
    ModuleLink &link = *call_link_of;
    if (len > MAX_FRAME) {
        return -EMSGSIZE;
    }
    if (request != link.request) {
        memcpy(link.request, request, len);
    }
    link.request_len = len;
    link.state.store(ModuleLink::Posted);
    // sent from here when the service could; otherwise its next step (1 ms poll) sends it
    (void)service_of->forward_posted();
    // the service completes every posted call within CALL_TIMEOUT_MS of posting
    while (link.state.load() != ModuleLink::Done) {
        wait_until(realtime_after_ms(10));
    }
    int32_t result = link.result;
    if (result > 0) {
        if (uint32_t(result) > cap) {
            result = -EMSGSIZE;
        } else if (reply != link.reply) {
            memcpy(reply, link.reply, size_t(result));
        }
    }
    link.state.store(ModuleLink::Idle);
    return result;
}

static void port_notify(void *platform, const uint8_t *frame, uint16_t len)
{
    (void)port_call(platform, frame, len, call_link_of->reply, MAX_FRAME);
}

static const HostPlatform host_platform {
    port_call, port_notify, AP_Module::binding_local_wire, AP_Module::binding_local_dispatch, nullptr
};

static bool own_context(void *context)
{
    return active != nullptr && context == active->token;
}

// a panicked module's thread waits here until the M4 process restarts
__attribute__((noreturn)) static void park_module()
{
    for (;;) {
        pause();
    }
}

extern "C" {

static uint8_t *host_api_allocate(void *context, uint32_t size, uint32_t alignment)
{
    return own_context(context) ? host_allocate(*active, size, alignment) : nullptr;
}

static void host_api_deallocate(void *context, uint8_t *allocation)
{
    if (!own_context(context) || host_deallocate(*active, allocation)) {
        return;
    }
    static const char text[] = "invalid deallocation";
    memcpy(active->panic_text, text, sizeof(text));
    host_panic_report(*active, host_platform);
    park_module();
}

// the M7 is another process here: its micros64() is not readable, the M4 process's clock stands in
static uint64_t host_api_time_us(void *)
{
    return monotonic_us();
}

static int32_t host_api_sleep_ms(void *context, uint32_t milliseconds)
{
    if (!own_context(context)) {
        return STATUS_INVALID;
    }
    if (!active->stop.load()) {
        const uint32_t ms = milliseconds < 60000U ? milliseconds : 60000U;
        if (ms == 0) {
            sched_yield();
        } else {
            const uint64_t end_us = monotonic_us() + uint64_t(ms) * 1000U;
            const struct timespec deadline = realtime_after_ms(ms);
            // a Done broadcast wakes it early too: wait out the rest unless stopped
            while (!active->stop.load() && monotonic_us() < end_us) {
                wait_until(deadline);
            }
        }
    }
    return active->stop.load() ? STATUS_STOPPED : STATUS_OK;
}

static uint8_t host_api_stop_requested(void *context)
{
    return own_context(context) ? uint8_t(active->stop.load()) : 1U;
}

static int32_t host_api_text(void *context, uint8_t severity, const uint8_t *message, uint32_t length)
{
    return own_context(context) ? host_text(*active, severity, message, length, host_platform) : STATUS_INVALID;
}

static void host_api_panic(void *context, const uint8_t *message, uint32_t length)
{
    if (active != nullptr) {
        host_panic_record(*active, own_context(context) ? message : nullptr, length, host_platform);
    }
    park_module();
}

static int32_t host_api_binding_call(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (active == nullptr) {
        return STATUS_INVALID;
    }
    const BindingCallArgs args { context, id, input, input_size, output, output_size };
    return host_binding_call(*active, args, host_platform);
}

extern const AP_ModuleHostApi module_host_api;
const AP_ModuleHostApi module_host_api {
    AP_MODULE_ABI_VERSION, sizeof(AP_ModuleHostApi), AP_MODULE_API_FINGERPRINT,
    host_api_allocate, host_api_deallocate, host_api_time_us, host_api_sleep_ms, host_api_stop_requested,
    host_api_text, host_api_panic, host_api_binding_call
};

}  // extern "C"

typedef int32_t (*ModuleStart)(const AP_ModuleHostApi *api, void *context);

static void *module_thread_main(void *)
{
    stack_t ss {};
    ss.ss_sp = alternate_stack;
    ss.ss_size = sizeof(alternate_stack);
    sigaltstack(&ss, nullptr);
    // the entry is an address in this process's code mapping (no Thumb bit on a host)
    const ModuleStart start = reinterpret_cast<ModuleStart>(uintptr_t(module_entry));
    const int32_t result = start(&module_host_api, active->token);
    module_result = result;
    active->state.store(uint8_t(ModuleStateKind::Returned));
    ModuleStateNotify notify {};
    fill_header(notify.call.header, MsgType::SystemCall, sizeof(notify), 0);
    notify.call.method = uint16_t(SystemMethod::ModuleState);
    notify.call.flags = CALL_FLAG_NOTIFY;
    notify.module = active->index;
    notify.state = uint8_t(ModuleStateKind::Returned);
    notify.result = result;
    port_notify(nullptr, reinterpret_cast<const uint8_t *>(&notify), sizeof(notify));
    module_has_returned = true;
    return nullptr;
}

void ModuleM4::module_port_init(ModuleLink &link, M4Service &service, void (*)())
{
    call_link_of = &link;
    service_of = &service;
}

void ModuleM4::module_port_wake()
{
    pthread_mutex_lock(&port_mutex);
    pthread_cond_broadcast(&port_cond);
    pthread_mutex_unlock(&port_mutex);
}

// entry: a uint32_t, as on the M4; the SITL code mapping must lie below 4 GB (module_port_code_map)
void ModuleM4::module_port_start(ModuleContext &context, const LoadLayout &layout, uint32_t entry, uint32_t)
{
    if (module_running && !module_has_returned) {
        return;   // one module thread at a time
    }
    if (module_running) {
        pthread_join(module_thread, nullptr);
        module_running = false;
    }
    module_has_returned = false;
    module_result = 0;
    module_entry = entry;
    context.token = module_token(context.index);
    context.stop.store(false);
    context.state.store(uint8_t(ModuleStateKind::Started));
    active = &context;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    const bool ok = pthread_attr_setstack(&attr, reinterpret_cast<void *>(uintptr_t(layout.stack_base)),
                                          layout.stack_top - layout.stack_base) == 0 &&
                    pthread_create(&module_thread, &attr, module_thread_main, nullptr) == 0;
    pthread_attr_destroy(&attr);
    if (!ok) {
        context.state.store(uint8_t(ModuleStateKind::Failed));
        active = nullptr;
        return;
    }
    module_running = true;
}

void ModuleM4::module_port_request_stop(ModuleContext &context)
{
    context.stop.store(true);
    module_port_wake();
}

bool ModuleM4::module_port_returned(const ModuleContext &context, int32_t &result)
{
    if (active != &context || !module_has_returned) {
        return false;
    }
    result = module_result;
    return true;
}

uint8_t *ModuleM4::module_port_code_map(uint32_t size)
{
    // the PR's 4 KiB permission boundaries; below 4 GB so that a uint32_t entry reaches it
    if (size == 0 || size > 16U * 1024U * 1024U || sysconf(_SC_PAGESIZE) != 4096) {
        return nullptr;
    }
    size = (size + 4095U) & ~4095U;
#ifdef MAP_32BIT
    const int low = MAP_32BIT;
#else
    const int low = 0;
#endif
    void *data = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | low, -1, 0);
    if (data == MAP_FAILED) {
        return nullptr;
    }
    if (uint64_t(uintptr_t(data)) + size > 0x100000000ULL) {
        munmap(data, size);
        return nullptr;
    }
    return static_cast<uint8_t *>(data);
}

bool ModuleM4::module_port_code_publish(uint8_t *code, uint32_t code_size)
{
    if (code == nullptr || code_size == 0 || code_size % 4096U != 0) {
        return false;
    }
    __builtin___clear_cache(reinterpret_cast<char *>(code), reinterpret_cast<char *>(code) + code_size);
    return mprotect(code, code_size, PROT_READ | PROT_EXEC) == 0;
}
