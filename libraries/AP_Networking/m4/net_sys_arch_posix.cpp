#include "net_sys_arch.h"

#include <errno.h>
#include <string.h>
#include <time.h>

static thread_local int net_errno;
// any per-thread object's address identifies the thread
static thread_local char thread_marker;

static pthread_once_t protect_once = PTHREAD_ONCE_INIT;
static pthread_mutex_t protect_mutex;

static void protect_init()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&protect_mutex, &attr);
    pthread_mutexattr_destroy(&attr);
}

// absolute CLOCK_REALTIME deadline for pthread_cond_timedwait
static timespec deadline_after(uint32_t timeout_ms)
{
    timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += timeout_ms / 1000;
    deadline.tv_nsec += long(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    return deadline;
}

// waits on cond until ready() holds; false on timeout (timeout_ms 0: forever)
template <typename Ready>
static bool wait_until(pthread_cond_t *cond, pthread_mutex_t *mutex, uint32_t timeout_ms, Ready ready)
{
    const timespec deadline = deadline_after(timeout_ms);
    while (!ready()) {
        if (timeout_ms == 0) {
            pthread_cond_wait(cond, mutex);
        } else if (pthread_cond_timedwait(cond, mutex, &deadline) == ETIMEDOUT) {
            return ready();
        }
    }
    return true;
}

void NetM4::thread_errno_set(int err)
{
    net_errno = err;
}

int NetM4::thread_errno_get()
{
    return net_errno;
}

void os_sem_init(OsSem *sem, int count)
{
    *sem = OsSem{PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, count};
}

void os_sem_signal(OsSem *sem)
{
    pthread_mutex_lock(&sem->mutex);
    sem->count++;
    pthread_cond_signal(&sem->signalled);
    pthread_mutex_unlock(&sem->mutex);
}

bool os_sem_wait_ms(OsSem *sem, uint32_t timeout_ms)
{
    pthread_mutex_lock(&sem->mutex);
    const bool taken = wait_until(&sem->signalled, &sem->mutex, timeout_ms, [sem] { return sem->count > 0; });
    if (taken) {
        sem->count--;
    }
    pthread_mutex_unlock(&sem->mutex);
    return taken;
}

void os_mutex_init(OsMutex *mutex)
{
    *mutex = OsMutex{PTHREAD_MUTEX_INITIALIZER};
}

void os_mutex_lock(OsMutex *mutex)
{
    pthread_mutex_lock(&mutex->m);
}

void os_mutex_unlock(OsMutex *mutex)
{
    pthread_mutex_unlock(&mutex->m);
}

void os_mbox_init(OsMbox *mbox, void **slots, int count)
{
    *mbox = OsMbox{PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER, PTHREAD_COND_INITIALIZER,
                   slots, count, 0, 0};
}

static void mbox_put(OsMbox *mbox, void *msg)
{
    mbox->slots[(mbox->head + mbox->used) % mbox->count] = msg;
    mbox->used++;
    pthread_cond_signal(&mbox->not_empty);
}

static void *mbox_take(OsMbox *mbox)
{
    void *msg = mbox->slots[mbox->head];
    mbox->head = (mbox->head + 1) % mbox->count;
    mbox->used--;
    pthread_cond_signal(&mbox->not_full);
    return msg;
}

void os_mbox_post(OsMbox *mbox, void *msg)
{
    pthread_mutex_lock(&mbox->mutex);
    wait_until(&mbox->not_full, &mbox->mutex, 0, [mbox] { return mbox->used < mbox->count; });
    mbox_put(mbox, msg);
    pthread_mutex_unlock(&mbox->mutex);
}

bool os_mbox_trypost(OsMbox *mbox, void *msg)
{
    pthread_mutex_lock(&mbox->mutex);
    const bool room = mbox->used < mbox->count;
    if (room) {
        mbox_put(mbox, msg);
    }
    pthread_mutex_unlock(&mbox->mutex);
    return room;
}

bool os_mbox_fetch_ms(OsMbox *mbox, void **msg, uint32_t timeout_ms)
{
    pthread_mutex_lock(&mbox->mutex);
    const bool got = wait_until(&mbox->not_empty, &mbox->mutex, timeout_ms, [mbox] { return mbox->used > 0; });
    if (got) {
        *msg = mbox_take(mbox);
    }
    pthread_mutex_unlock(&mbox->mutex);
    return got;
}

bool os_mbox_tryfetch(OsMbox *mbox, void **msg)
{
    pthread_mutex_lock(&mbox->mutex);
    const bool got = mbox->used > 0;
    if (got) {
        *msg = mbox_take(mbox);
    }
    pthread_mutex_unlock(&mbox->mutex);
    return got;
}

const void *os_self()
{
    return &thread_marker;
}

uint32_t os_now_ms()
{
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return uint32_t(uint64_t(now.tv_sec) * 1000U + uint64_t(now.tv_nsec) / 1000000U);
}

struct ThreadStart {
    void (*fn)(void *);
    void *arg;
};

static void *thread_entry(void *start)
{
    const ThreadStart *entry = static_cast<const ThreadStart *>(start);
    entry->fn(entry->arg);
    return nullptr;
}

// the M4's threads, each started at most once, as net_sys_arch_chibios.cpp
static const char *const thread_names[] = { "tcpip", "net_port", "ppp_rx" };
static ThreadStart thread_starts[sizeof(thread_names) / sizeof(thread_names[0])];
static bool thread_started[sizeof(thread_names) / sizeof(thread_names[0])];

bool os_thread_start(const char *name, int priority, void (*fn)(void *), void *arg)
{
    (void)priority;
    size_t i = 0;
    while (i < sizeof(thread_names) / sizeof(thread_names[0]) && strcmp(name, thread_names[i]) != 0) {
        i++;
    }
    if (i == sizeof(thread_names) / sizeof(thread_names[0])) {
        return false;
    }
    os_protect_enter();
    const bool taken = thread_started[i];
    thread_started[i] = true;
    os_protect_exit();
    if (taken) {
        return false;
    }
    ThreadStart *start = &thread_starts[i];
    *start = ThreadStart{fn, arg};
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    pthread_t thread;
    const bool started = pthread_create(&thread, &attr, thread_entry, start) == 0;
    pthread_attr_destroy(&attr);
    return started;
}

void os_protect_enter()
{
    pthread_once(&protect_once, protect_init);
    pthread_mutex_lock(&protect_mutex);
}

void os_protect_exit()
{
    pthread_mutex_unlock(&protect_mutex);
}
