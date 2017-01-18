#include "pthread_impl.h"

volatile size_t __pthread_tsd_size = sizeof(void*) * PTHREAD_KEYS_MAX;
void* __pthread_tsd_main[PTHREAD_KEYS_MAX] = {};

static void (*volatile keys[PTHREAD_KEYS_MAX])(void*);

static void nodtor(void* dummy) {}

int __pthread_key_create(pthread_key_t* k, void (*dtor)(void*)) {
    unsigned i = (uintptr_t)&k / 16 % PTHREAD_KEYS_MAX;
    unsigned j = i;

    if (!dtor)
        dtor = nodtor;
    do {
        if (!a_cas_p(keys + j, 0, (void*)dtor)) {
            *k = j;
            return 0;
        }
    } while ((j = (j + 1) % PTHREAD_KEYS_MAX) != i);
    return EAGAIN;
}

int __pthread_key_delete(pthread_key_t k) {
    keys[k] = 0;
    return 0;
}

void __pthread_tsd_run_dtors(void) {
    pthread_t self = __pthread_self();
    int i, j, not_finished = self->tsd_used;
    for (j = 0; not_finished && j < PTHREAD_DESTRUCTOR_ITERATIONS; j++) {
        not_finished = 0;
        for (i = 0; i < PTHREAD_KEYS_MAX; i++) {
            if (self->tsd[i] && keys[i]) {
                void* tmp = self->tsd[i];
                self->tsd[i] = 0;
                keys[i](tmp);
                not_finished = 1;
            }
        }
    }
}

weak_alias(__pthread_key_delete, pthread_key_delete);
weak_alias(__pthread_key_create, pthread_key_create);
