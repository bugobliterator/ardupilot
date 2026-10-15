#include "net_sys_arch.h"

#include <string.h>

static_assert(sizeof(msg_t) == sizeof(void *), "mailbox slots hold pointers");

// static working areas, one per network thread; a thread never exits, so each starts once
static THD_WORKING_AREA(tcpip_wa, 2048);
static THD_WORKING_AREA(port_wa, 2048);
static THD_WORKING_AREA(ppp_rx_wa, 1536);

static const struct {
    const char *name;
    stkalign_t *area;
    size_t size;
} thread_areas[] = {
    { "tcpip", tcpip_wa, sizeof(tcpip_wa) },
    { "net_port", port_wa, sizeof(port_wa) },
    { "ppp_rx", ppp_rx_wa, sizeof(ppp_rx_wa) },
};
static bool thread_started[sizeof(thread_areas) / sizeof(thread_areas[0])];

static syssts_t protect_status;
static uint32_t protect_depth;

static sysinterval_t interval_ms(uint32_t timeout_ms)
{
    return timeout_ms == 0 ? TIME_INFINITE : TIME_MS2I(timeout_ms);
}

void NetM4::thread_errno_set(int err)
{
    chThdGetSelfX()->net_errno = err;
}

int NetM4::thread_errno_get()
{
    return chThdGetSelfX()->net_errno;
}

void os_sem_init(OsSem *sem, int count)
{
    chSemObjectInit(&sem->sem, (cnt_t)count);
}

void os_sem_signal(OsSem *sem)
{
    chSemSignal(&sem->sem);
}

bool os_sem_wait_ms(OsSem *sem, uint32_t timeout_ms)
{
    return chSemWaitTimeout(&sem->sem, interval_ms(timeout_ms)) == MSG_OK;
}

void os_mutex_init(OsMutex *mutex)
{
    chMtxObjectInit(&mutex->m);
}

void os_mutex_lock(OsMutex *mutex)
{
    chMtxLock(&mutex->m);
}

void os_mutex_unlock(OsMutex *mutex)
{
    chMtxUnlock(&mutex->m);
}

void os_mbox_init(OsMbox *mbox, void **slots, int count)
{
    chMBObjectInit(&mbox->mb, (msg_t *)slots, (size_t)count);
}

void os_mbox_post(OsMbox *mbox, void *msg)
{
    (void)chMBPostTimeout(&mbox->mb, (msg_t)msg, TIME_INFINITE);
}

bool os_mbox_trypost(OsMbox *mbox, void *msg)
{
    return chMBPostTimeout(&mbox->mb, (msg_t)msg, TIME_IMMEDIATE) == MSG_OK;
}

bool os_mbox_fetch_ms(OsMbox *mbox, void **msg, uint32_t timeout_ms)
{
    msg_t value;
    if (chMBFetchTimeout(&mbox->mb, &value, interval_ms(timeout_ms)) != MSG_OK) {
        return false;
    }
    *msg = (void *)value;
    return true;
}

bool os_mbox_tryfetch(OsMbox *mbox, void **msg)
{
    msg_t value;
    if (chMBFetchTimeout(&mbox->mb, &value, TIME_IMMEDIATE) != MSG_OK) {
        return false;
    }
    *msg = (void *)value;
    return true;
}

const void *os_self()
{
    return chThdGetSelfX();
}

uint32_t os_now_ms()
{
    return (uint32_t)TIME_I2MS(chVTGetSystemTimeX());
}

bool os_thread_start(const char *name, int priority, void (*fn)(void *), void *arg)
{
    for (size_t i = 0; i < sizeof(thread_areas) / sizeof(thread_areas[0]); i++) {
        if (strcmp(name, thread_areas[i].name) != 0) {
            continue;
        }
        chSysLock();
        const bool taken = thread_started[i];
        thread_started[i] = true;
        chSysUnlock();
        if (taken) {
            return false;
        }
        chThdCreateStatic(thread_areas[i].area, thread_areas[i].size, (tprio_t)(NORMALPRIO + priority), fn, arg);
        return true;
    }
    return false;
}

// nests: the outermost enter saves the interrupt state, the matching exit restores it
void os_protect_enter()
{
    const syssts_t status = chSysGetStatusAndLockX();
    if (protect_depth++ == 0) {
        protect_status = status;
    }
}

void os_protect_exit()
{
    if (--protect_depth == 0) {
        chSysRestoreStatusX(protect_status);
    }
}
