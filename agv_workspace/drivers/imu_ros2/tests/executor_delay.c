#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static atomic_bool injected = false;
static atomic_bool delayed = false;
static atomic_uint publications = 0;

int rcl_wait(void *wait_set, int64_t timeout) {
    int (*original)(void *, int64_t) = dlsym(RTLD_NEXT, "rcl_wait");
    if (!atomic_exchange(&injected, true)) {
        const struct timespec delay = {0, 400000000};
        fprintf(stderr, "TEST executor wait delay begin\n");
        atomic_store(&delayed, true);
        nanosleep(&delay, NULL);
        atomic_store(&delayed, false);
        fprintf(stderr, "TEST executor wait delay end imu_publications=%u\n",
                atomic_load(&publications));
    }
    return original(wait_set, timeout);
}

int rcl_publish(const void *publisher, const void *message, void *allocation) {
    int (*original)(const void *, const void *, void *) = dlsym(RTLD_NEXT, "rcl_publish");
    const char *(*topic_name)(const void *) = dlsym(RTLD_NEXT, "rcl_publisher_get_topic_name");
    const char *topic = topic_name(publisher);
    int result = original(publisher, message, allocation);
    if (result == 0 && topic && strcmp(topic, "/imu") == 0 && atomic_load(&delayed))
        atomic_fetch_add(&publications, 1);
    return result;
}
