/*
  SITL stand-in for the Cortex-M4 that runs native Rust modules: a second
  process attached to the M7 SITL's CubeFramework POSIX segment (two-process
  mode). It runs the same M4Service as the M4 image. Faults are signals: the handler records
  them like the M4 fault handler and waits for the M7's restart grant; a
  restart is an execv of this program (the M7 sends SIGUSR1).
 */

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/Posix.h>
#include <CubePilotFW/pal/PosixChannel.h>
#include <AP_RustM4_Layout.h>
#include <AP_RustM4_ServeWait.h>

#include "../m4_service.h"
#include "../module_host.h"
#include "../module_port.h"

using namespace RustM4;
using CubePilotFW::Channel;
namespace PeerHeader = CubePilotFW::PeerHeader;

static uint8_t *window;
static char self_path[PATH_MAX];
static char **self_argv;
static pid_t parent;
static uint32_t restart_grant_at_start;
static ModuleLink call_link;   // not `link`: <unistd.h> declares link()
static M4Service service;
static pthread_mutex_t service_mutex = PTHREAD_MUTEX_INITIALIZER;   // M4Platform::lock
static ModuleHost module_host;

static void sleep_ms(long ms)
{
    struct timespec ts { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&ts, nullptr);
}

static uint64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000U + uint64_t(ts.tv_nsec) / 1000U;
}

static uint32_t millis_now()
{
    return uint32_t(now_us() / 1000U);
}

static uint32_t cycles_now()
{
    return uint32_t(now_us());   // one "cycle" per microsecond
}

static void lock_service()
{
    pthread_mutex_lock(&service_mutex);
}

static void unlock_service()
{
    pthread_mutex_unlock(&service_mutex);
}

// SIG_IGN survives execv and discards pending copies: a SIGUSR1 racing the
// restart must not hit the new image before it installs its handler
static void ignore_restart_signal()
{
    struct sigaction ign {};
    ign.sa_handler = SIG_IGN;
    sigaction(SIGUSR1, &ign, nullptr);
}

static_assert(std::atomic<uint32_t>::is_always_lock_free, "window atomics are read in signal handlers");

static void on_restart_signal(int)
{
    SupervisorBlock *block = supervisor_block(window);
    if (block->run_permit.load() == 0) {
        _exit(0);   // the M7 turned the M4 off: park for good
    }
    // the M7 stores the grant before signalling; a late or repeated signal is ignored
    if (block->restart_grant.load() == restart_grant_at_start) {
        return;
    }
    ignore_restart_signal();
    execv(self_path, self_argv);
    _exit(71);
}

static void on_fault_signal(int sig, siginfo_t *info, void *)
{
    PeerHeader::recordFault(window, PeerHeader::Core::M4, uint32_t(sig), 0,
                            uint32_t(uintptr_t(info->si_addr)), 0);
    SupervisorBlock *block = supervisor_block(window);
    while (block->restart_grant.load() == restart_grant_at_start) {
        if (getppid() != parent) {
            _exit(0);
        }
        sleep_ms(1);
    }
    ignore_restart_signal();
    execv(self_path, self_argv);
    _exit(71);
}

static void inject_fault(FaultKind kind)
{
    volatile uint32_t spin = 0;
    switch (kind) {
    case FaultKind::HardFault:
        __builtin_trap();
    case FaultKind::MemManage:
        *reinterpret_cast<volatile uint32_t *>(uintptr_t(8)) = 0;
        break;
    case FaultKind::Stall:
        while (getppid() == parent) {
            spin = spin + 1;    // the restart signal still arrives
        }
        _exit(0);
    case FaultKind::Lockup: {
        sigset_t restart;
        sigemptyset(&restart);
        sigaddset(&restart, SIGUSR1);
        sigprocmask(SIG_BLOCK, &restart, nullptr);
        while (getppid() == parent) {
            spin = spin + 1;    // ignores the restart signal, like interrupts off
        }
        _exit(0);
    }
    default:
        break;
    }
}

// the loader's platform: the service's Config and Stats names, the module port's start
static bool host_config(void *, ConfigReply &reply)
{
    if (!service.config_ready()) {
        return false;
    }
    reply = service.config_reply();
    return true;
}

static void host_binding_name(void *, uint32_t id, const char *name)
{
    service.set_binding_name(id, name);
}

static bool host_publish(void *, uint8_t *code, uint32_t size)
{
    return module_port_code_publish(code, size);
}

static void host_start(void *, ModuleContext &module, const LoadLayout &layout, uint32_t entry, uint32_t rw_base)
{
    module_port_start(module, layout, entry, rw_base);
}

static void host_request_stop(void *, ModuleContext &module)
{
    module_port_request_stop(module);
}

// M4Platform::stop_module, on the service thread: the loader runs its one module
static void stop_module(uint16_t module)
{
    if (module == module_host.module().index) {
        module_host.request_stop();
    }
}

// the module area below 4 GB (the layout and the entry are uint32_t, as on the M4), its top aligned
// as the code window needs: at the M4's own address where that is free (MAP_32BIT is not honoured
// under every x86-64 emulation), else anywhere below 4 GB; null if it cannot be mapped
static uint8_t *map_module_area()
{
#ifdef MAP_FIXED_NOREPLACE
    void *fixed = mmap(reinterpret_cast<void *>(uintptr_t(AP_RUST_M4_MODULE_AREA_BASE)), AP_RUST_M4_MODULE_AREA_SIZE,
                       PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (fixed == reinterpret_cast<void *>(uintptr_t(AP_RUST_M4_MODULE_AREA_BASE))) {
        return static_cast<uint8_t *>(fixed);
    }
    if (fixed != MAP_FAILED) {
        munmap(fixed, AP_RUST_M4_MODULE_AREA_SIZE);   // a kernel that ignores the flag
    }
#endif
    uint8_t *mapping = module_port_code_map(AP_RUST_M4_MODULE_AREA_SIZE + CODE_WINDOW_MAX);
    if (mapping == nullptr) {
        return nullptr;
    }
    const uintptr_t top = (uintptr_t(mapping) + AP_RUST_M4_MODULE_AREA_SIZE + CODE_WINDOW_MAX) &
                          ~uintptr_t(CODE_WINDOW_MAX - 1U);
    return reinterpret_cast<uint8_t *>(top - AP_RUST_M4_MODULE_AREA_SIZE);
}

static void install_handlers()
{
    // an execv from a handler keeps that handler's mask: start from an empty one
    sigset_t none;
    sigemptyset(&none);
    sigprocmask(SIG_SETMASK, &none, nullptr);

    static uint8_t alternate_stack[64 * 1024];
    stack_t ss {};
    ss.ss_sp = alternate_stack;
    ss.ss_size = sizeof(alternate_stack);
    sigaltstack(&ss, nullptr);

    struct sigaction fault {};
    fault.sa_sigaction = on_fault_signal;
    fault.sa_flags = SA_SIGINFO | SA_ONSTACK;
    static const int fault_signals[] = { SIGILL, SIGTRAP, SIGSEGV, SIGBUS, SIGFPE, SIGABRT };
    for (int sig : fault_signals) {
        sigaction(sig, &fault, nullptr);
    }
    struct sigaction restart {};
    restart.sa_handler = on_restart_signal;
    sigaction(SIGUSR1, &restart, nullptr);
}

int main(int argc, char **argv)
{
    (void)argc;
    parent = getppid();
    self_argv = argv;
    // descriptors inherited from the M7 SITL (sockets, logs) are not ours
    for (int fd = 3; fd < 1024; fd++) {
        close(fd);
    }
    if (realpath(argv[0], self_path) == nullptr) {
        snprintf(self_path, sizeof(self_path), "%s", argv[0]);
    }
    const char *name = getenv("CUBEFW_SHM_NAME");
    if (name == nullptr) {
        fprintf(stderr, "rust_m4: CUBEFW_SHM_NAME not set (started by the M7 SITL)\n");
        return 1;
    }
    static Channel channel;
    const int ret = CubePilotFW::PosixChannel::start(channel, Channel::Core::M4, DOORBELL_GROUP, CHANNEL_WINDOW_SIZE);
    if (ret != 0) {
        fprintf(stderr, "rust_m4: PosixChannel::start: %d\n", ret);
        return 1;
    }
    window = cubefw_posix_attach(name);   // the mapping start() made
    SupervisorBlock *block = supervisor_block(window);
    if (block->magic.load() != SUPERVISOR_MAGIC) {
        fprintf(stderr, "rust_m4: supervisor block not set up\n");
        return 1;
    }
    restart_grant_at_start = block->restart_grant.load();
    install_handlers();
    PeerHeader::publish(window, PeerHeader::Core::M4, PROTOCOL_VERSION, CONTRACT_HASH,
                        restart_grant_at_start);
    // RUST_M4_INJECT 12: the session the M7 marked faults before the M7 reads its header
    if (restart_grant_at_start != 0 && block->fault_after_publish_grant.load() == restart_grant_at_start) {
        inject_fault(FaultKind::HardFault);
    }

    // the built-in thread-path selftest (RUST_M4_INJECT 17) is the M4 image's only
    // time_us is this process's own clock: no M7 millis() to take
    const M4Platform platform { millis_now, cycles_now, 1, inject_fault, module_port_wake, nullptr, nullptr,
                                lock_service, unlock_service, stop_module, &module_port_call_probe() };
    module_port_init(call_link, service, nullptr);
    service.init(channel, window, call_link, platform);
    uint8_t *const area = map_module_area();
    // the PR's x86-64 identity and host callback margin (Module.cpp); no area: no module loads
    const ModuleHostPlatform host_platform {
        &call_link, area, uint32_t(uintptr_t(area)), AP_RUST_M4_MODULE_AREA_SIZE,
        { 62, AP_RUST_ABI_VERSION, 2, 0, AP_RUST_API_FINGERPRINT }, 16384, nullptr, host_config,
        host_binding_name, nullptr, host_publish, host_start, host_request_stop
    };
    module_host.init(host_platform);
    if (area == nullptr) {
        fprintf(stderr, "rust_m4: no module area below 4 GB\n");
    }
    uint32_t housekeeping_us = cycles_now();
    bool slept = true;   // the first pass runs the housekeeping
    while (true) {
        if (getppid() != parent || block->run_permit.load() != 1) {
            _exit(0);   // not exit(): a module thread may still run on the statics
        }
        // as the M4 image: the frames after every wake, the rest after a sleep or once a millisecond
        service.drain_step();
        const uint32_t now_us = cycles_now();
        const bool housekeeping = housekeeping_due(now_us, housekeeping_us, HOUSEKEEPING_PERIOD_US, slept);
        if (housekeeping) {
            housekeeping_us = now_us;
            service.housekeeping_step();
        }
        if (area != nullptr && (housekeeping || module_host.loading())) {
            module_host.step();
        }
        // POSIX HSEM has no interrupt: poll with a bounded sleep
        slept = channel.prepareSleep();
        if (slept) {
            sleep_ms(1);
        }
        channel.wake();
    }
}
