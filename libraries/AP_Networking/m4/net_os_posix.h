#pragma once

#include <pthread.h>

struct OsSem {
    pthread_mutex_t mutex;
    pthread_cond_t signalled;
    int count;
};

struct OsMutex {
    pthread_mutex_t m;
};

// a ring of count slots over the array the pool entry owns
struct OsMbox {
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
    void **slots;
    int count;
    int head;
    int used;
};
