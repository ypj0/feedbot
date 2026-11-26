#define _GNU_SOURCE

#include <dlfcn.h>
#include <errno.h>
#include <execinfo.h>
#include <pthread.h>
#include <stdint.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static _Atomic(int (*)(pthread_mutex_t *)) real_pthread_mutex_lock;
static _Atomic(int (*)(pthread_mutex_t *, const struct timespec *)) real_pthread_mutex_timedlock;
static _Atomic(int (*)(pthread_mutex_t *, clockid_t, const struct timespec *)) real_pthread_mutex_clocklock;
static _Atomic(int (*)(pthread_cond_t *, pthread_mutex_t *)) real_pthread_cond_wait;
static _Atomic(int (*)(pthread_cond_t *, pthread_mutex_t *, const struct timespec *)) real_pthread_cond_timedwait;
static _Atomic(int (*)(pthread_cond_t *, pthread_mutex_t *, clockid_t, const struct timespec *)) real_pthread_cond_clockwait;

static _Thread_local int in_probe;

static void fail_load(const char *symbol)
{
    static const char prefix[] = "imu_wait_probe: failed to resolve ";
    static const char newline[] = "\n";
    size_t symbol_length = 0;

    while (symbol[symbol_length] != '\0') {
        ++symbol_length;
    }
    (void)write(STDERR_FILENO, prefix, sizeof(prefix) - 1);
    (void)write(STDERR_FILENO, symbol, symbol_length);
    (void)write(STDERR_FILENO, newline, sizeof(newline) - 1);
    _exit(127);
}

#define RESOLVE(name)                                                         \
    do {                                                                      \
        if (real_##name == NULL) real_##name = dlsym(RTLD_NEXT, #name);        \
        if (real_##name == NULL) {                                            \
            fail_load(#name);                                                 \
        }                                                                     \
    } while (0)

static int64_t elapsed_nanoseconds(const struct timespec *start,
                                   const struct timespec *end)
{
    return (int64_t)(end->tv_sec - start->tv_sec) * 1000000000LL +
           (end->tv_nsec - start->tv_nsec);
}

static void report_wait(const char *function_name, const void *lock,
                        const struct timespec *start,
                        const struct timespec *end)
{
    int64_t elapsed_ns = elapsed_nanoseconds(start, end);
    void *frames[64];
    int frame_count;

    if (elapsed_ns <= 100000000LL) {
        return;
    }

    in_probe = 1;
    (void)dprintf(STDERR_FILENO,
                  "imu_wait_probe: function=%s thread=%ld lock=%p "
                  "duration_ms=%.3f start_monotonic=%lld.%09ld "
                  "end_monotonic=%lld.%09ld\n",
                  function_name, (long)syscall(SYS_gettid), lock,
                  (double)elapsed_ns / 1000000.0,
                  (long long)start->tv_sec, start->tv_nsec,
                  (long long)end->tv_sec, end->tv_nsec);
    frame_count = backtrace(frames, (int)(sizeof(frames) / sizeof(frames[0])));
    backtrace_symbols_fd(frames, frame_count, STDERR_FILENO);
    in_probe = 0;
}

#define WRAP_WAIT(call_expression, function_name, lock_pointer)               \
    do {                                                                      \
        struct timespec start;                                                \
        struct timespec end;                                                  \
        int have_start;                                                       \
        int result;                                                           \
        int saved_errno;                                                      \
                                                                                \
        if (in_probe || syscall(SYS_gettid) != getpid()) {                    \
            return (call_expression);                                         \
        }                                                                     \
        have_start = clock_gettime(CLOCK_MONOTONIC, &start) == 0;             \
        result = (call_expression);                                            \
        saved_errno = errno;                                                  \
        if (have_start && clock_gettime(CLOCK_MONOTONIC, &end) == 0) {        \
            report_wait((function_name), (lock_pointer), &start, &end);       \
        }                                                                     \
        errno = saved_errno;                                                  \
        return result;                                                        \
    } while (0)

int pthread_mutex_lock(pthread_mutex_t *mutex)
{
    RESOLVE(pthread_mutex_lock);
    WRAP_WAIT(real_pthread_mutex_lock(mutex), "pthread_mutex_lock", mutex);
}

int pthread_mutex_timedlock(pthread_mutex_t *mutex,
                            const struct timespec *absolute_time)
{
    RESOLVE(pthread_mutex_timedlock);
    WRAP_WAIT(real_pthread_mutex_timedlock(mutex, absolute_time),
              "pthread_mutex_timedlock", mutex);
}

int pthread_mutex_clocklock(pthread_mutex_t *mutex, clockid_t clock_id,
                            const struct timespec *absolute_time)
{
    RESOLVE(pthread_mutex_clocklock);
    WRAP_WAIT(real_pthread_mutex_clocklock(mutex, clock_id, absolute_time),
              "pthread_mutex_clocklock", mutex);
}

int pthread_cond_wait(pthread_cond_t *condition, pthread_mutex_t *mutex)
{
    RESOLVE(pthread_cond_wait);
    WRAP_WAIT(real_pthread_cond_wait(condition, mutex), "pthread_cond_wait",
              mutex);
}

int pthread_cond_timedwait(pthread_cond_t *condition, pthread_mutex_t *mutex,
                           const struct timespec *absolute_time)
{
    RESOLVE(pthread_cond_timedwait);
    WRAP_WAIT(real_pthread_cond_timedwait(condition, mutex, absolute_time),
              "pthread_cond_timedwait", mutex);
}

int pthread_cond_clockwait(pthread_cond_t *condition, pthread_mutex_t *mutex,
                           clockid_t clock_id,
                           const struct timespec *absolute_time)
{
    RESOLVE(pthread_cond_clockwait);
    WRAP_WAIT(real_pthread_cond_clockwait(condition, mutex, clock_id,
                                         absolute_time),
              "pthread_cond_clockwait", mutex);
}
