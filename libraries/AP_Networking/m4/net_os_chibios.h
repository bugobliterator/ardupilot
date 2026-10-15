#pragma once

#include "ch.h"

struct OsSem {
    semaphore_t sem;
};

struct OsMutex {
    mutex_t m;
};

struct OsMbox {
    mailbox_t mb;
};
