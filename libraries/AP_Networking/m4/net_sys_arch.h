#pragma once

/*
  Static lwIP sys_arch for the M4 network service: fixed pools of semaphores, mutexes,
  mailboxes and thread working areas, no malloc. The pool bookkeeping is shared; each
  platform supplies the OS objects (net_sys_arch_chibios.cpp, net_sys_arch_posix.cpp).
  Starting point: the Network Baseline probe's probe_sys_arch.c.
 */

#include <stddef.h>
#include <stdint.h>
#include "lwip/sys.h"

namespace NetM4 {

static const uint8_t NUM_SEMS = 24;
static const uint8_t NUM_MUTEXES = 4;
static const uint8_t NUM_MBOXES = 21;
static const uint8_t TCPIP_SLOTS = 16;
static const uint8_t UDP_RECV_SLOTS = 8;
static const uint8_t LUA_UDP_RECV_SLOTS = 4;
static const uint8_t TCP_RECV_SLOTS = 4;
static const uint8_t ACCEPT_SLOTS = 4;
static_assert(DEFAULT_TCP_RECVMBOX_SIZE == TCP_RECV_SLOTS, "every TCP netconn gets 4 receive slots");
static_assert(DEFAULT_UDP_RECVMBOX_SIZE == UDP_RECV_SLOTS, "the M7's UDP netconns keep 8");

int mbox_slots_for(int requested, bool lua_thread);
int mbox_depth(const sys_mbox_t *mbox);
void set_lua_thread(const void *thread);
bool current_is_lua_thread();
void thread_errno_set(int err);
int thread_errno_get();

}  // namespace NetM4

// one OS object per kind: the platform's header defines the three structs, its .cpp the functions
#if defined(NET_M4_POSIX)
#include "net_os_posix.h"      // pthread mutex and condition variables
#else
#include "net_os_chibios.h"    // semaphore_t, mutex_t, mailbox_t
#endif
void os_sem_init(OsSem *sem, int count);
void os_sem_signal(OsSem *sem);
bool os_sem_wait_ms(OsSem *sem, uint32_t timeout_ms);   // 0: forever
void os_mutex_init(OsMutex *mutex);
void os_mutex_lock(OsMutex *mutex);
void os_mutex_unlock(OsMutex *mutex);
// thread context only: no os_* call is ISR-safe (no sys_mbox_trypost_fromisr)
void os_mbox_init(OsMbox *mbox, void **slots, int count);
void os_mbox_post(OsMbox *mbox, void *msg);
bool os_mbox_trypost(OsMbox *mbox, void *msg);
bool os_mbox_fetch_ms(OsMbox *mbox, void **msg, uint32_t timeout_ms);   // 0: forever
bool os_mbox_tryfetch(OsMbox *mbox, void **msg);
const void *os_self();
uint32_t os_now_ms();
// priority is relative: NORMALPRIO + priority on ChibiOS, ignored on POSIX
bool os_thread_start(const char *name, int priority, void (*fn)(void *), void *arg);
void os_protect_enter();
void os_protect_exit();
