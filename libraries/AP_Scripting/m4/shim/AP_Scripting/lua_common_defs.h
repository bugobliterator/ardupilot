#pragma once
/*
  M4 Lua build: lua_common_defs.h without AP_HAL. The M7 lists and serves the
  scripts, so the directory below only feeds luaconf.h's unused LUA_LDIR.
  The functions are defined by m4/m4_lua.cpp.
 */
#define SCRIPTING_DIRECTORY "/APM/scripts"

int lua_get_current_env_ref(void);
const char *lua_get_modules_path(void);
void lua_abort(void) __attribute__((noreturn));

// lua_str2number() of the M4 (luaconf.h, AP_LUA_MAX_NUMBER_TEXT): strtof, or not a number
// for a longer text, so that strtof never takes newlib's reserve for a huge numeral
#ifdef __cplusplus
extern "C" {
#endif
float lua_bounded_str2number(const char *s, char **p);
#ifdef __cplusplus
}
#endif
