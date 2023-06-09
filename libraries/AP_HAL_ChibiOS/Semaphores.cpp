/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include <hal.h>
#include "AP_HAL_ChibiOS.h"

#if CH_CFG_USE_MUTEXES == TRUE

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

uint16_t Semaphore::global_counter = 0;
static mutex_t sem_chain_lock;
struct Semaphore::sem_chain_trunk *Semaphore::sem_chain = NULL;

// semaphore class
class WithSemChainSemaphore {
public:
    WithSemChainSemaphore() {
        chMtxLock(&sem_chain_lock);
    }
    ~WithSemChainSemaphore() {
        chMtxUnlock(&sem_chain_lock);
    }
};

// constructor
Semaphore::Semaphore()
{
    static_assert(sizeof(_lock) >= sizeof(mutex_t), "invalid mutex size");
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxObjectInit(mtx);
    chMtxObjectInit(&sem_chain_lock);
    sem_chain_lock.obj = NULL;
    mtx->obj = this;
    sem_index = global_counter++;
}

bool Semaphore::give()
{
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxUnlock(mtx);
    return true;
}

#define WITH_SEM_CHAIN() WithSemChainSemaphore _with_sem_chain

void Semaphore::update_sem_chain()
{
    WITH_SEM_CHAIN();
    volatile thread_reference_t thd = chThdGetSelfX();
    // skip the first mutex, which is the one we are holding for this
    // method
    mutex_t *mtx_head = thd->mtxlist->next;
    if (mtx_head->next == NULL) {
        // don't care about an orphan mutex
        return;
    }
    volatile sem_chain_trunk *trunk = sem_chain;
    // here we search through the list of thread->semaphore_chains
    while (trunk) {
        if (trunk->thd != thd) {
            // not the thread we are looking for
            trunk = trunk->next;
            continue;
        }
        if (trunk->head_link == NULL) {
            // this thread has no semaphore chain
            trunk = trunk->next;
            continue;
        }
        volatile sem_chain_branch *branch = trunk->head_link;
        volatile sem_chain_branch *prev_branch = NULL;
        mutex_t *mtx = mtx_head;
        while (mtx) {
            Semaphore* sem = (Semaphore*)mtx->obj;
            if (sem == NULL) {
                // we can't analyse this mutex
                mtx = mtx->next;
                continue;
            }
            if (branch == NULL) {
                prev_branch->next_link = (sem_chain_branch*)malloc(sizeof(sem_chain_branch));
                if (prev_branch->next_link == NULL) {
                    return;
                }
                branch = prev_branch->next_link;
                branch->sem = sem;
                branch->next_link = NULL;
            }
            // check if this branch already exists in the chain
            if (sem != branch->sem) {
                // this branch is not in the chain
                break;
            }
            mtx = mtx->next;
            prev_branch = branch;
            branch = branch->next_link;
        }
        if (mtx == NULL) {
            return;
        }
        trunk = trunk->next;
    }

    // restart with the trunk
    mutex_t *mtx = mtx_head;
    if (sem_chain == NULL) {
        sem_chain = (sem_chain_trunk*)malloc(sizeof(sem_chain_trunk));
        if (sem_chain == NULL) {
            return;
        }
        sem_chain->thd = thd;
        sem_chain->next = NULL;
        sem_chain->head_link = (sem_chain_branch*)malloc(sizeof(sem_chain_branch));
        if (sem_chain->head_link == NULL) {
            return;
        }
        sem_chain->head_link->sem = (Semaphore*)mtx->obj;
        sem_chain->head_link->next_link = NULL;
        mtx = mtx->next;

        sem_chain_branch *branch = sem_chain->head_link;
        while (mtx) {
            Semaphore* sem = (Semaphore*)mtx->obj;
            if (sem == NULL) {
                // we can't analyse this mutex
                mtx = mtx->next;
                continue;
            }
            if (branch->next_link == NULL) {
                branch->next_link = (sem_chain_branch*)malloc(sizeof(sem_chain_branch));
                if (branch->next_link == NULL) {
                    return;
                }
                branch->next_link->sem = sem;
                branch->next_link->next_link = NULL;
            }
            mtx = mtx->next;
            branch = branch->next_link;
        }
        return;
    }

    trunk = sem_chain;
    while (trunk->next) {
        trunk = trunk->next;
    }
    trunk->next = (sem_chain_trunk*)malloc(sizeof(sem_chain_trunk));
    if (trunk->next == NULL) {
        return;
    }
    trunk->next->thd = thd;
    trunk->next->next = NULL;
    trunk->next->head_link = (sem_chain_branch*)malloc(sizeof(sem_chain_branch));
    if (trunk->next->head_link == NULL) {
        return;
    }
    trunk->next->head_link->sem = (Semaphore*)mtx->obj;
    trunk->next->head_link->next_link = NULL;
    mtx = mtx->next;
    while (mtx)
    {
        Semaphore* sem = (Semaphore*)mtx->obj;
        if (sem == NULL) {
            // we can't analyse this mutex
            mtx = mtx->next;
            continue;
        }
        if (trunk->next->head_link->next_link == NULL) {
            trunk->next->head_link->next_link = (sem_chain_branch*)malloc(sizeof(sem_chain_branch));
            if (trunk->next->head_link->next_link == NULL) {
                return;
            }
            trunk->next->head_link->next_link->sem = sem;
            trunk->next->head_link->next_link->next_link = NULL;
        }
        mtx = mtx->next;
    }
}

void Semaphore::get_sem_chain(ExpandingString &str)
{
    WITH_SEM_CHAIN();
    sem_chain_trunk *trunk = sem_chain;
    while (trunk) {
        sem_chain_branch *branch = trunk->head_link;
        str.printf("thread(%s): ", trunk->thd->name?trunk->thd->name:"NA");
        while (branch) {
            if (branch->sem) {
                str.printf("%u", branch->sem->sem_index);
                if (branch->sem->filename) {
                    str.printf("(%s,%d)", branch->sem->filename, branch->sem->line);
                }
            } else {
                str.printf("NULL");
            }
            branch = branch->next_link;
            if (branch) {
                str.printf("->");
            }
        }
        str.printf("\n");
        trunk = trunk->next;
    }
}

bool Semaphore::take(uint32_t timeout_ms)
{
    mutex_t *mtx = (mutex_t *)_lock;
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        chMtxLock(mtx);
        update_sem_chain();
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            update_sem_chain();
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    mutex_t *mtx = (mutex_t *)_lock;
    return chMtxTryLock(mtx);
}

bool Semaphore::check_owner(void)
{
    mutex_t *mtx = (mutex_t *)_lock;
    return mtx->owner == chThdGetSelfX();
}

void Semaphore::assert_owner(void)
{
    osalDbgAssert(check_owner(), "owner");
}

#endif // CH_CFG_USE_MUTEXES
