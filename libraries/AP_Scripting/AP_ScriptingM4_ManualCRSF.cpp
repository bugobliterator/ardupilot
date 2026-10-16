/*
  The CRSF menu manual bindings of AP_Scripting_CRSFMenu.cpp, ported to the
  remote adapter (gen-bindings -r remote_ready_manual[]). Menus and parameters
  are owned objects in the M7's pool; the menus they name are AP_CRSF_Telem's,
  cleared when the session ends, as the in-process thread clears them after a
  run. AP::crsf_telem() is checked: in-process the bindings dereference it. The
  limits AP_CRSF_Telem assumes but does not check are kept here, before its code
  runs (crsf_add_menu(), crsf_add_parameter()), and the M7 builds the menus
  itself (AP_ScriptingM4_CRSF.h).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include "AP_Scripting_CRSFMenu.h"
#include "AP_ScriptingM4_CRSF.h"
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

// scr_sem for an M4 call, waited for at most CRSF_LOCK_TIMEOUT_MS: the CRSF thread holds it while
// it builds one frame. AP_CRSF_Telem's own take inside the call is then a recursive one. Nothing
// that can raise a Lua error runs while it is held (ap_longjmp would skip the give)
class CRSFLock {
public:
    explicit CRSFLock(AP_CRSF_Telem &telem) :
        _sem(telem.get_semaphore()),
        _held(_sem.take(CRSF_LOCK_TIMEOUT_MS)) {}
    ~CRSFLock()
    {
        if (_held) {
            _sem.give();
        }
    }
    bool held() const { return _held; }
    CLASS_NO_COPY(CRSFLock);

private:
    HAL_Semaphore &_sem;
    const bool _held;
};

const CRSFAllocators crsf_heap { malloc, calloc };

AP_CRSF_Telem::ScriptedMenu *crsf_make_menu(AP_CRSF_Telem &telem, AP_CRSF_Telem::ScriptedMenu *parent,
                                            const char *name, const CRSFAllocators &alloc)
{
    // the host thread or its slow worker, one at a time, is the only one that adds to a menu
    if (parent != nullptr && !crsf_menu_has_room(parent->num_params)) {
        return nullptr;
    }
    AP_CRSF_Telem::ScriptedMenu *menu = NEW_NOTHROW AP_CRSF_Telem::ScriptedMenu();
    if (menu == nullptr) {
        return nullptr;
    }
    // the destructor frees the name, the parameter array and the data of num_params parameters
    menu->num_params = 0;
    menu->next_menu = nullptr;
    menu->parent_id = parent == nullptr ? 0 : parent->id;
    const uint8_t name_size = AP_CRSF_Telem::MAX_SCRIPTED_MENU_NAME_LEN + 1;
    char *menu_name = (char *)alloc.name(name_size);
    menu->name = menu_name;
    menu->params = (AP_CRSF_Telem::ScriptedParameter *)alloc.params(CRSF_PARAMETERS_PER_MENU,
                                                                    sizeof(AP_CRSF_Telem::ScriptedParameter));
    if (menu_name == nullptr || menu->params == nullptr) {
        delete menu;
        return nullptr;
    }
    strncpy(menu_name, name, name_size - 1);
    menu_name[name_size - 1] = 0;

    CRSFLock lock(telem);
    if (!lock.held()) {
        delete menu;
        return nullptr;
    }
    AP_CRSF_Telem::ScriptedMenu *tail = &telem.scripted_menus;
    while (tail->next_menu != nullptr) {
        tail = tail->next_menu;
    }
    // AP_CRSF_Telem::ScriptedMenu::add_menu()'s numbering: the menu's parameters take the ids after it
    const uint8_t first_id = AP_CRSF_Telem::SCRIPTED_MENU_START_ID;
    const uint8_t menu_ids = AP_CRSF_Telem::MAX_SCRIPTED_MENU_SIZE + 1;
    menu->id = tail->id == 0 ? first_id : uint8_t(tail->id + menu_ids);
    if (parent != nullptr) {
        // the parent's link to it: a parameter of no data with the menu's id
        AP_CRSF_Telem::ScriptedParameter &link = parent->params[parent->num_params];
        link.id = menu->id;
        link.parent_id = parent->id;
        link.length = 0;
        link.data = nullptr;
        parent->num_params++;
    }
    tail->next_menu = menu;
    return menu;
}

AP_CRSF_Telem::ScriptedParameter *crsf_make_parameter(AP_CRSF_Telem &telem, AP_CRSF_Telem::ScriptedMenu &menu,
                                                      size_t length, const char *data)
{
    if (length == 0 || length > AP_CRSF_Telem::MAX_SCRIPTED_PARAMETER_SIZE || !crsf_menu_has_room(menu.num_params)) {
        return nullptr;
    }
    char *copy = (char *)malloc(length);
    if (copy == nullptr) {
        return nullptr;
    }
    memcpy(copy, data, length);

    CRSFLock lock(telem);
    if (!lock.held()) {
        free(copy);
        return nullptr;
    }
    // the array holds CRSF_PARAMETERS_PER_MENU: queued radio requests that point into it stay valid
    AP_CRSF_Telem::ScriptedParameter &param = menu.params[menu.num_params];
    param.id = uint8_t(menu.id + menu.num_params + 1);
    param.parent_id = menu.id;
    param.length = uint16_t(length);
    param.data = copy;
    menu.num_params++;
    return &param;
}

void crsf_clear_menus(AP_CRSF_Telem &telem)
{
    // blocking: the menus must go. The CRSF thread holds scr_sem while it builds one frame
    WITH_SEMAPHORE(telem.get_semaphore());
    // radio requests point at the parameters of the menus
    telem.inbound_params.clear();
    telem.outbound_params.clear();
    telem.ready_params.clear();
    telem.clear_menus();
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
        AP_CRSF_Telem::ScriptedMenu* menu = crsf_make_menu(*telem, nullptr, cut);
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
    AP_CRSF_Telem *telem = crsf_telem(L);
    uint8_t data = 0;
    {
        // no event while the CRSF thread holds the menus
        CRSFLock lock(*telem);
        if (lock.held()) {
            data = static_cast<uint8_t>(telem->get_menu_event(
                    events,
                    param,
                    payload));
        }
    }

    lua_pushinteger(L, param);
    // the radio frame's length, which can exceed the payload buffer
    lua_pushlstring(L, (const char*)payload.payload, MIN(payload.payload_length, sizeof(payload.payload)));
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
    AP_CRSF_Telem *telem = crsf_telem(L);
    uint8_t count = 0;
    {
        // no event while the CRSF thread holds the menus
        CRSFLock lock(*telem);
        if (lock.held()) {
            count = telem->peek_menu_event(param, payload, events);
        }
    }

    lua_pushinteger(L, count);
    lua_pushinteger(L, param);
    // the radio frame's length, which can exceed the payload buffer
    lua_pushlstring(L, (const char*)payload.payload, MIN(payload.payload_length, sizeof(payload.payload)));
    lua_pushinteger(L, events);
    return 4;
}

// crsf:send_write_response(payload): lua_CRSF_send_response()
int lua_CRSF_send_response(lua_State *L)
{
    binding_argcheck(L, 2);

    size_t len = 0;
    const char * payload= luaL_checklstring(L, 2, &len);
    AP_CRSF_Telem *telem = crsf_telem(L);
    bool data = false;
    {
        // false while the CRSF thread holds the menus, as when there is no request to answer
        CRSFLock lock(*telem);
        if (lock.held()) {
            data = static_cast<bool>(telem->send_write_response(
                    len,
                    payload));
        }
    }

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

    // a menu handle names a menu of AP_CRSF_Telem, which exists then
    AP_CRSF_Telem *telem = AP::crsf_telem();
    return crsf_add_parameter(ud->num_params(), [L, ud, telem, len, data]() {
        // create Lua object first in case of errors so we don't leak the param
        auto *p = new_CRSFParameter(L);

        // nil for no data or more than 255 bytes (in-process: a cut or uninitialised parameter)
        AP_CRSF_Telem::ScriptedParameter* param = crsf_make_parameter(*telem, *ud->get_menu(), len, data);
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

    // a menu handle names a menu of AP_CRSF_Telem, which exists then
    AP_CRSF_Telem *telem = AP::crsf_telem();
    return crsf_add_menu(menu_budget, crsf_menu_has_room(ud->num_params()), name, [L, ud, telem](const char *cut) {
        // create Lua object first in case of errors so we don't leak the menu
        auto *m = new_CRSFMenu(L);
        AP_CRSF_Telem::ScriptedMenu* menu = crsf_make_menu(*telem, ud->get_menu(), cut);
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
        AP_CRSF_Telem::ScriptedMenu* menu = crsf_make_menu(*telem, nullptr, cut);
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
        crsf_clear_menus(*telem);
    }
    menu_budget.new_session();
#endif
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
