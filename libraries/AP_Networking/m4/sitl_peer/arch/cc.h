/*
  lwIP platform for m4_net_peer (host)
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define LWIP_ERRNO_STDINCLUDE 1
#define LWIP_PLATFORM_DIAG(x) do { printf x; } while (0)
#define LWIP_PLATFORM_ASSERT(x) do { fprintf(stderr, "m4_net_peer: lwIP assert %s\n", x); abort(); } while (0)
#define LWIP_RAND() ((uint32_t)rand())
