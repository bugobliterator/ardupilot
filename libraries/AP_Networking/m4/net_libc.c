/*
  Privileged replacement for newlib's atoi (lwIP netif.c, netif_find): newlib's writes errno
  through _impure_ptr, which lives in Lua RAM, so a privileged caller must not use it.
 */
#include <stdlib.h>
#include "lwip/sys.h"

// a C object: the M4 Makefile passes the M4 include directory in USE_COPT as well as USE_CPPOPT
#if !defined(AP_NETWORKING_M4_SYS_ARCH)
#error "M4 lwIP picked up the M7 arch/sys_arch.h"
#endif

int atoi(const char *text)
{
    // unsigned: an over-long number wraps instead of overflowing a signed int
    unsigned value = 0;
    int sign = 1;
    while (*text == ' ' || (*text >= '\t' && *text <= '\r')) {
        text++;
    }
    if (*text == '-' || *text == '+') {
        sign = (*text == '-') ? -1 : 1;
        text++;
    }
    while (*text >= '0' && *text <= '9') {
        value = value * 10U + (unsigned)(*text - '0');
        text++;
    }
    return sign < 0 ? (int)(0U - value) : (int)value;
}
