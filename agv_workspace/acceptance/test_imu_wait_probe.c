#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t condition = PTHREAD_COND_INITIALIZER;
static pthread_barrier_t barrier;

static void *hold_mutex(void *unused)
{
    (void)unused;
    assert(pthread_mutex_lock(&mutex) == 0);
    pthread_barrier_wait(&barrier);
    usleep(150000);
    assert(pthread_mutex_unlock(&mutex) == 0);
    return NULL;
}

int main(void)
{
    pthread_t worker;
    struct timespec deadline;
    assert(pthread_barrier_init(&barrier, NULL, 2) == 0);
    assert(pthread_create(&worker, NULL, hold_mutex, NULL) == 0);
    pthread_barrier_wait(&barrier);
    errno = EDOM;
    assert(pthread_mutex_lock(&mutex) == 0);
    assert(errno == EDOM);
    assert(clock_gettime(CLOCK_MONOTONIC, &deadline) == 0);
    deadline.tv_nsec += 150000000;
    if (deadline.tv_nsec >= 1000000000) {
        deadline.tv_nsec -= 1000000000;
        ++deadline.tv_sec;
    }
    errno = ERANGE;
    assert(pthread_cond_clockwait(&condition, &mutex, CLOCK_MONOTONIC, &deadline) == ETIMEDOUT);
    assert(errno == ERANGE);
    assert(pthread_mutex_unlock(&mutex) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(pthread_barrier_destroy(&barrier) == 0);
    puts("PASS: mutex/condition wait return values and errno preserved");
    return 0;
}
