#pragma once

/*
  The scripted CRSF menus of an M4 session, built by the M7 itself rather than
  by AP_CRSF_Telem::ScriptedMenu::add_menu()/add_parameter(), which an M4 script
  must not reach:
  - add_parameter() reallocates the menu's parameter array on every call, while
    radio requests queued in AP_CRSF_Telem point into it, and leaves the data
    pointer of an empty parameter (a sub-menu's link) uninitialised for the
    destructor to free;
  - add_menu() and the ScriptedMenu constructor use allocations unchecked.
  Here a menu's parameter array holds CRSF_PARAMETERS_PER_MENU from the start
  and never moves, every allocation is checked, and the menus change only under
  AP_CRSF_Telem's scr_sem, which the CRSF thread holds while it reads them.
  Scripting host thread only.
 */

#include <AP_RCTelemetry/AP_RCTelemetry_config.h>
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED && AP_CRSF_SCRIPTING_ENABLED

#include <stddef.h>
#include <stdlib.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>

namespace ScriptingM4Remote {

// an M4 call waits at most this for AP_CRSF_Telem's scr_sem, which the CRSF thread holds while it
// builds one frame; past it the call returns its refusal value
static const uint32_t CRSF_LOCK_TIMEOUT_MS = 30;

// malloc() for the menu name and calloc() for the parameter array; a test passes one that fails
struct CRSFAllocators {
    void *(*name)(size_t size);
    void *(*params)(size_t count, size_t size);
};
extern const CRSFAllocators crsf_heap;

// a menu for the script: a root (parent nullptr) or a sub-menu, linked by a new parameter of its
// parent. nullptr if the parent is full or the M7 heap has no room; nothing is linked then
AP_CRSF_Telem::ScriptedMenu *crsf_make_menu(AP_CRSF_Telem &telem, AP_CRSF_Telem::ScriptedMenu *parent,
                                            const char *name, const CRSFAllocators &alloc = crsf_heap);

// a parameter of length bytes: nullptr for 0 bytes (a sub-menu link, which AP_CRSF_Telem reads as
// a folder) or more than AP_CRSF_Telem's 255, in a full menu, or if the M7 heap has no room
AP_CRSF_Telem::ScriptedParameter *crsf_make_parameter(AP_CRSF_Telem &telem, AP_CRSF_Telem::ScriptedMenu &menu,
                                                      size_t length, const char *data);

// the session ends: the radio requests queued for its menus go, then the menus
void crsf_clear_menus(AP_CRSF_Telem &telem);

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED && AP_CRSF_SCRIPTING_ENABLED
