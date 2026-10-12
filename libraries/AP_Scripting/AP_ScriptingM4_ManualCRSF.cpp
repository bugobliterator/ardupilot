/*
  The CRSF menu manual bindings of AP_Scripting_CRSFMenu.cpp, ported to the
  remote adapter (gen-bindings -r remote_ready_manual[]). Menus and parameters
  are owned objects in the M7's pool; the menus they name are AP_CRSF_Telem's,
  cleared when the session ends, as the in-process thread clears them after a
  run. AP::crsf_telem() is checked: in-process the bindings dereference it. The
  limits AP_CRSF_Telem assumes but does not check are kept here, before its code
  runs (crsf_add_menu(), crsf_add_parameter()).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include "AP_Scripting_CRSFMenu.h"
#include "AP_ScriptingM4_Remote.h"

namespace ScriptingM4Remote {

#if AP_CRSF_SCRIPTING_ENABLED

static_assert(CRSF_MENUS_PER_SESSION == AP_CRSF_Telem::MAX_SCRIPTED_MENUS, "CRSF menu limit");
static_assert(CRSF_PARAMETERS_PER_MENU == AP_CRSF_Telem::MAX_SCRIPTED_MENU_SIZE, "CRSF parameter limit");
static_assert(CRSF_NAME_LENGTH + 1 == AP_CRSF_Telem::MAX_SCRIPTED_MENU_NAME_LEN, "CRSF name length");

// generated (lua_remote_dispatch.cpp)
CRSFMenu *check_CRSFMenu(lua_State *L, int arg);
CRSFMenu *new_CRSFMenu(lua_State *L);
CRSFParameter *check_CRSFParameter(lua_State *L, int arg);
CRSFParameter *new_CRSFParameter(lua_State *L);

// the menus the session made: crsf_end_session() clears them, and only then, because
// AP::crsf_telem() makes CRSF telemetry when disarmed, which a session end must not do
static CRSFMenuBudget menu_budget;

static AP_CRSF_Telem *crsf_telem(lua_State *L)
{
    return crsf_required(AP::crsf_telem(), L);
}

// CRSFMenu(name): lua_CRSF_new_menu()
int lua_CRSF_new_menu(lua_State *L)
{
    binding_argcheck(L, 1);

    const char * name = luaL_checkstring(L, 1);
    AP_CRSF_Telem *telem = crsf_telem(L);

    // the session's menus are used up: the in-process error for a menu that could not be made
    const bool made = crsf_add_menu(menu_budget, true, name, [L, telem](const char *cut) {
        // create Lua object first in case of errors so we don't leak the menu
        auto *m = new_CRSFMenu(L);
        AP_CRSF_Telem::ScriptedMenu* menu = telem->add_menu(cut);
        if (menu == nullptr) {
            return false;
        }
        *m = CRSFMenu(menu);
        return true;
    });
    if (!made) {
        return luaL_error(L, "out of memory");
    }
    return 1;
}

// crsf:get_menu_event(events): lua_CRSF_get_menu_event()
int lua_CRSF_get_menu_event(lua_State *L)
{
    binding_argcheck(L, 2);

    const uint8_t events = get_uint8_t(L, 2);
    uint8_t param = 0;
    AP_CRSF_Telem::ScriptedPayload payload {};
    const uint8_t data = static_cast<uint8_t>(crsf_telem(L)->get_menu_event(
            events,
            param,
            payload));

    lua_pushinteger(L, param);
    lua_pushlstring(L, (const char*)payload.payload, payload.payload_length);
    lua_pushinteger(L, data);
    return 3;
}

// crsf:peek_menu_event(): lua_CRSF_peek_menu_event()
int lua_CRSF_peek_menu_event(lua_State *L)
{
    binding_argcheck(L, 1);

    uint8_t param = 0;
    uint8_t events = 0;
    AP_CRSF_Telem::ScriptedPayload payload {};
    const uint8_t count = crsf_telem(L)->peek_menu_event(param, payload, events);

    lua_pushinteger(L, count);
    lua_pushinteger(L, param);
    lua_pushlstring(L, (const char*)payload.payload, payload.payload_length);
    lua_pushinteger(L, events);
    return 4;
}

// crsf:send_write_response(payload): lua_CRSF_send_response()
int lua_CRSF_send_response(lua_State *L)
{
    binding_argcheck(L, 2);

    size_t len = 0;
    const char * payload= luaL_checklstring(L, 2, &len);
    const bool data = static_cast<bool>(crsf_telem(L)->send_write_response(
            len,
            payload));

    lua_pushboolean(L, data);
    return 1;
}

// menu:add_parameter(data): lua_CRSF_add_parameter(). Nil in a menu that holds
// CRSF_PARAMETERS_PER_MENU
int lua_CRSF_add_parameter(lua_State *L)
{
    binding_argcheck(L, 2);

    CRSFMenu* ud = check_CRSFMenu(L, 1);
    size_t len = 0;
    const char * data = luaL_checklstring(L, 2, &len);

    return crsf_add_parameter(ud->num_params(), [L, ud, len, data]() {
        // create Lua object first in case of errors so we don't leak the param
        auto *p = new_CRSFParameter(L);

        AP_CRSF_Telem::ScriptedParameter* param = ud->add_parameter(len, data);
        if (param == nullptr) {
            return false;
        }
        *p = CRSFParameter(ud->get_menu(), param);
        return true;
    }) ? 1 : 0;
}

// menu:add_menu(name): lua_CRSF_add_menu(). Nil past the session's menus or in a menu that holds
// CRSF_PARAMETERS_PER_MENU (the sub-menu is one of its parameters)
int lua_CRSF_add_menu(lua_State *L)
{
    binding_argcheck(L, 2);

    CRSFMenu* ud = check_CRSFMenu(L, 1);
    const char * name = luaL_checkstring(L, 2);

    return crsf_add_menu(menu_budget, crsf_menu_has_room(ud->num_params()), name, [L, ud](const char *cut) {
        // create Lua object first in case of errors so we don't leak the menu
        auto *m = new_CRSFMenu(L);
        AP_CRSF_Telem::ScriptedMenu* menu = ud->add_menu(cut);
        if (menu == nullptr) {
            return false;
        }
        *m = CRSFMenu(menu);
        return true;
    }) ? 1 : 0;
}

// crsf:add_menu(name): lua_CRSF_add_root_menu(). Nil past the session's menus
int lua_CRSF_add_root_menu(lua_State *L)
{
    binding_argcheck(L, 2);

    const char * name = luaL_checkstring(L, 2);
    AP_CRSF_Telem *telem = crsf_telem(L);

    return crsf_add_menu(menu_budget, true, name, [L, telem](const char *cut) {
        // create Lua object first in case of errors so we don't leak the menu
        auto *m = new_CRSFMenu(L);
        AP_CRSF_Telem::ScriptedMenu* menu = telem->add_menu(cut);
        if (menu == nullptr) {
            return false;
        }
        *m = CRSFMenu(menu);
        return true;
    }) ? 1 : 0;
}

// param:data(): lua_CRSF_param_data()
int lua_CRSF_param_data(lua_State *L)
{
    binding_argcheck(L, 1);

    CRSFParameter * ud = check_CRSFParameter(L, 1);
    AP_CRSF_Telem::ScriptedParameter* param = ud->get_parameter();
    if (param == nullptr) {
        return luaL_error(L, "internal error: %s is null", "CRSFParameter");
    }

    lua_pushlstring(L, param->data, param->length);
    return 1;
}

#endif  // AP_CRSF_SCRIPTING_ENABLED

void crsf_end_session()
{
#if AP_CRSF_SCRIPTING_ENABLED
    if (!menu_budget.any()) {
        return;
    }
    // the menus exist, so CRSF telemetry does: this makes nothing
    AP_CRSF_Telem *telem = AP::crsf_telem();
    if (telem != nullptr) {
        telem->clear_menus();
    }
    menu_budget.new_session();
#endif
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
