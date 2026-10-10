#include <stddef.h>
#include <stdint.h>
#include <string.h>

static volatile uint32_t pal_microseconds;

uint32_t cubefw_pal_timeNowUs(void)
{
    return ++pal_microseconds;  // monotonic only
}

void cubefw_pal_sleepUs(uint32_t us)
{
    (void)us;
}

void cubefw_pal_criticalEnter(void)
{
    __asm volatile("cpsid i" ::: "memory");
}

void cubefw_pal_criticalExit(void)
{
    __asm volatile("cpsie i" ::: "memory");
}

void cubefw_pal_shMemcpy(void *dst, const void *src, size_t len)
{
    memcpy(dst, src, len);
}
