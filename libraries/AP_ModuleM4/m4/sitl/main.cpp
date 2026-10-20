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
#include <time.h>
#include <unistd.h>

#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/Posix.h>
#include <CubePilotFW/pal/PosixChannel.h>
#include <AP_ModuleM4_Layout.h>

#include "../m4_service.h"

using namespace ModuleM4;
using CubePilotFW::Channel;
namespace PeerHeader = CubePilotFW::PeerHeader;

static uint8_t *window;
static char self_path[PATH_MAX];
static char **self_argv;
static pid_t parent;
static uint32_t restart_grant_at_start;
static ModuleLink call_link;   // not `link`: <unistd.h> declares link(); no module thread posts on it yet
static M4Service service;
static pthread_mutex_t service_mutex = PTHREAD_MUTEX_INITIALIZER;   // M4Platform::lock

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
        fprintf(stderr, "module_m4: CUBEFW_SHM_NAME not set (started by the M7 SITL)\n");
        return 1;
    }
    static Channel channel;
    const int ret = CubePilotFW::PosixChannel::start(channel, Channel::Core::M4, DOORBELL_GROUP, CHANNEL_WINDOW_SIZE);
    if (ret != 0) {
        fprintf(stderr, "module_m4: PosixChannel::start: %d\n", ret);
        return 1;
    }
    window = cubefw_posix_attach(name);   // the mapping start() made
    SupervisorBlock *block = supervisor_block(window);
    if (block->magic.load() != SUPERVISOR_MAGIC) {
        fprintf(stderr, "module_m4: supervisor block not set up\n");
        return 1;
    }
    restart_grant_at_start = block->restart_grant.load();
    install_handlers();
    PeerHeader::publish(window, PeerHeader::Core::M4, PROTOCOL_VERSION, CONTRACT_HASH,
                        restart_grant_at_start);
    // MOD_M4_INJECT 12: the session the M7 marked faults before the M7 reads its header
    if (restart_grant_at_start != 0 && block->fault_after_publish_grant.load() == restart_grant_at_start) {
        inject_fault(FaultKind::HardFault);
    }

    const M4Platform platform { millis_now, cycles_now, 1, inject_fault, nullptr, lock_service,
                                unlock_service };
    service.init(channel, window, call_link, platform);
    while (true) {
        if (getppid() != parent || block->run_permit.load() != 1) {
            _exit(0);   // not exit(): a module thread may still run on the statics
        }
        service.step();
        // POSIX HSEM has no interrupt: poll with a bounded sleep
        if (channel.prepareSleep()) {
            sleep_ms(1);
        }
        channel.wake();
    }
}
