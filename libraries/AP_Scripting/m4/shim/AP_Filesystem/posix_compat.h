#pragma once
/*
  M4 Lua build: the C library's stdio stands in for AP_Filesystem. On the M4
  image fopen() fails (newlib nosys), so luaL_loadfile() and the io library
  are unavailable until the IO and Manual Bindings sub-phase proxies files
  through the M7; scripts are loaded through m4:read_script.
 */
#include <stdio.h>
#include <stdlib.h>
