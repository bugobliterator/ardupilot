#pragma once
// sys_prot_t comes from AP's arch/cc.h (libraries/AP_Networking/lwip_hal/include)
struct sys_sem;
struct sys_mutex;
struct sys_mbox;
typedef struct sys_sem *sys_sem_t;
typedef struct sys_mutex *sys_mutex_t;
typedef struct sys_mbox *sys_mbox_t;
typedef void *sys_thread_t;
#define sys_sem_valid(sem) (*(sem) != NULL)
#define sys_sem_set_invalid(sem) do { *(sem) = NULL; } while (0)
#define sys_mutex_valid(mutex) (*(mutex) != NULL)
#define sys_mutex_set_invalid(mutex) do { *(mutex) = NULL; } while (0)
#define sys_mbox_valid(mbox) (*(mbox) != NULL)
#define sys_mbox_set_invalid(mbox) do { *(mbox) = NULL; } while (0)
