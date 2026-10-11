#pragma once

/*
  What lua_remote_local.cpp (generated) and generator/gen/generated_cpp_deps.h
  need on the M4, in place of the AP headers of the M7 build.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <new>
#include "m4_bindings.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
// no in-process binding statistics on the M4: its service times the remote calls
#define BINDING_FN(id, fn) fn
// MAV_SEVERITY_WARNING, for userdata_zero_arg_check()
#define MAV_SEVERITY_WARNING 4
