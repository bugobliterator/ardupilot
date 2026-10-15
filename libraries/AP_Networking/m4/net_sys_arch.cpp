#include "net_sys_arch.h"

#include <string.h>
#include "lwip/opt.h"
#include "lwip/err.h"

#if !defined(AP_NETWORKING_M4_SYS_ARCH)
#error "M4 lwIP picked up the M7 arch/sys_arch.h"
#endif

struct sys_sem {
    OsSem os;
    bool used;
};

struct sys_mutex {
    OsMutex os;
    bool used;
};

struct sys_mbox {
    OsMbox os;
    bool used;
    int count;
    void *slots[NetM4::TCPIP_SLOTS];
};

static sys_sem sems[NetM4::NUM_SEMS];
static sys_mutex mutexes[NetM4::NUM_MUTEXES];
static sys_mbox mboxes[NetM4::NUM_MBOXES];
static const void *lua_thread;
static uint32_t rand_state;

// takes the first free entry of a pool, or nullptr when every entry is in use
template <typename Entry, size_t N>
static Entry *take_entry(Entry (&pool)[N])
{
    os_protect_enter();
    for (auto &entry : pool) {
        if (!entry.used) {
            entry.used = true;
            os_protect_exit();
            return &entry;
        }
    }
    os_protect_exit();
    return nullptr;
}

template <typename Entry>
static void release_entry(Entry *entry)
{
    os_protect_enter();
    entry->used = false;
    os_protect_exit();
}

int NetM4::mbox_slots_for(int requested, bool lua_thread_asks)
{
    int slots = requested < 1 ? 1 : requested;
    // a UDP netconn the Lua thread creates (lwIP asks DEFAULT_UDP_RECVMBOX_SIZE); TCP is 4 for all
    if (lua_thread_asks && slots == DEFAULT_UDP_RECVMBOX_SIZE) {
        slots = LUA_UDP_RECV_SLOTS;
    }
    return slots > TCPIP_SLOTS ? TCPIP_SLOTS : slots;
}

int NetM4::mbox_depth(const sys_mbox_t *mbox)
{
    return (*mbox)->count;
}

void NetM4::set_lua_thread(const void *thread)
{
    os_protect_enter();
    lua_thread = thread;
    os_protect_exit();
}

bool NetM4::current_is_lua_thread()
{
    os_protect_enter();
    const bool is_lua = lua_thread != nullptr && lua_thread == os_self();
    os_protect_exit();
    return is_lua;
}

// as lwIP's default set_errno: a success (0) leaves errno as it is
extern "C" void net_m4_set_errno(int err)
{
    if (err != 0) {
        NetM4::thread_errno_set(err);
    }
}

// a failed lwIP assertion faults the M4: the fault handler records it and the M7 restarts the M4
extern "C" void net_m4_assert(const char *file, int line)
{
    (void)file;
    (void)line;
    __builtin_trap();
}

// lwIP's LWIP_RAND (arch/cc.h): xorshift32, not newlib's rand, whose state is in Lua RAM
extern "C" unsigned int lwip_port_rand(void)
{
    os_protect_enter();
    if (rand_state == 0) {
        rand_state = os_now_ms() | 1U;
    }
    uint32_t x = rand_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rand_state = x;
    os_protect_exit();
    return x;
}

// the pools are zero-initialised statics
void sys_init(void)
{
}

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    sys_sem *entry = take_entry(sems);
    if (entry == nullptr) {
        *sem = nullptr;
        return ERR_MEM;
    }
    os_sem_init(&entry->os, count);
    *sem = entry;
    return ERR_OK;
}

void sys_sem_free(sys_sem_t *sem)
{
    release_entry(*sem);
}

void sys_sem_signal(sys_sem_t *sem)
{
    os_sem_signal(&(*sem)->os);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    const uint32_t start = os_now_ms();
    if (!os_sem_wait_ms(&(*sem)->os, timeout)) {
        return SYS_ARCH_TIMEOUT;
    }
    return os_now_ms() - start;
}

err_t sys_mutex_new(sys_mutex_t *mutex)
{
    sys_mutex *entry = take_entry(mutexes);
    if (entry == nullptr) {
        *mutex = nullptr;
        return ERR_MEM;
    }
    os_mutex_init(&entry->os);
    *mutex = entry;
    return ERR_OK;
}

void sys_mutex_free(sys_mutex_t *mutex)
{
    release_entry(*mutex);
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
    os_mutex_lock(&(*mutex)->os);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
    os_mutex_unlock(&(*mutex)->os);
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    sys_mbox *entry = take_entry(mboxes);
    if (entry == nullptr) {
        *mbox = nullptr;
        return ERR_MEM;
    }
    entry->count = NetM4::mbox_slots_for(size, NetM4::current_is_lua_thread());
    os_mbox_init(&entry->os, entry->slots, entry->count);
    *mbox = entry;
    return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
    release_entry(*mbox);
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    os_mbox_post(&(*mbox)->os, msg);
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
    return os_mbox_trypost(&(*mbox)->os, msg) ? ERR_OK : ERR_MEM;
}

// the ISR path is unsupported on the M4: no os_* call is ISR-safe
err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg)
{
    (void)mbox;
    (void)msg;
    net_m4_assert(__FILE__, __LINE__);
    return ERR_MEM;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    const uint32_t start = os_now_ms();
    void *value;
    if (!os_mbox_fetch_ms(&(*mbox)->os, &value, timeout)) {
        return SYS_ARCH_TIMEOUT;
    }
    if (msg != nullptr) {
        *msg = value;
    }
    return os_now_ms() - start;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
    void *value;
    if (!os_mbox_tryfetch(&(*mbox)->os, &value)) {
        return SYS_MBOX_EMPTY;
    }
    if (msg != nullptr) {
        *msg = value;
    }
    return 0;
}

// only the tcpip thread is lwIP's; the service starts the other network threads itself
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
    (void)stacksize;
    (void)prio;
    if (name == nullptr || strcmp(name, TCPIP_THREAD_NAME) != 0) {
        LWIP_ASSERT("sys_thread_new: only the tcpip thread", 0);
        return nullptr;
    }
    // tcpip_init ignores the result: without the thread every API call would block forever
    if (!os_thread_start(name, TCPIP_THREAD_PRIO, thread, arg)) {
        LWIP_ASSERT("sys_thread_new: tcpip thread not started", 0);
        return nullptr;
    }
    // lwIP only compares the handle with NULL
    return const_cast<char *>(name);
}

u32_t sys_now(void)
{
    return os_now_ms();
}

// PPP's magic-number seed (magic.c)
u32_t sys_jiffies(void)
{
    return os_now_ms();
}

// os_protect_enter() nests, as lwIP requires of SYS_ARCH_PROTECT
sys_prot_t sys_arch_protect(void)
{
    os_protect_enter();
    return 0;
}

void sys_arch_unprotect(sys_prot_t level)
{
    (void)level;
    os_protect_exit();
}

extern "C" void sys_mark_tcpip_thread(void)
{
}

// LWIP_ASSERT_CORE_LOCKED(): empty, as the M7's lwip_hal sys_arch
extern "C" void sys_check_core_locking(void)
{
}
