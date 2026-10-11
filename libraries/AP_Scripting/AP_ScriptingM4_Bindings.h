#pragma once

/*
  The generated bindings as the M4 sees them. lua_remote_tables.cpp (written
  by gen-bindings -r) lists per type its rows, {Lua name, method id, flags},
  which the M4 sandbox turns into remote_stub closures on first use. The
  m4:bindings image the M7 sends at every M4 start says which methods and
  types this firmware compiled in and carries the enum values.
  C++11, no ArduPilot dependencies: the M4 image, the SITL M4 process, the
  M7 and the host unit tests include it.
 */

#include <stdint.h>
#include <string.h>
#include "AP_ScriptingM4_Rpc.h"
#include <AP_Scripting/lua_remote_contract.h>

namespace ScriptingM4 {

enum class BindingKind : uint8_t {
    Singleton = 0,   // a global userdata; its methods get their self as nil
    Blob = 1,        // pure-data userdata held as the M7's wire bytes; methods write the self back
    Handle = 2,      // an M7 object of this M4 session
    Local = 3,       // implemented on the M4 (uint32_t, uint64_t)
    Global = 4,      // the global functions (millis, micros)
};

// BindingRow::flags
static const uint8_t ROW_HAS_SELF = 0x01;      // argument 1 is a singleton: sent as nil
static const uint8_t ROW_WRITES_SELF = 0x02;   // the reply's last value is the updated self blob
static const uint8_t ROW_OPERATOR = 0x04;      // a metamethod of a blob type (__add, ...)
static const uint8_t ROW_CREATION = 0x08;      // the global constructor of a blob type (the type's name)

struct BindingType {
    const char *name;        // global name and metatable name (__name)
    uint8_t kind;            // BindingKind
    uint8_t enum_count;
    uint16_t first_row;
    uint16_t row_count;
    uint16_t first_enum;
};

struct BindingRow {
    const char *name;        // the Lua key: method, field, alias or metamethod; "new" for the constructor
    uint16_t method;
    uint8_t flags;
    uint8_t type;            // id of the owning type
};

// lua_remote_tables.cpp
extern const BindingType binding_types[];          // by type id - 1
extern const BindingRow binding_rows[];
extern const char *const binding_enum_names[];
extern const uint16_t binding_method_rows[];       // by method id - FIRST_BINDING_METHOD
static const uint16_t NO_ROW = 0xFFFF;             // an id no remote row uses

// the m4:bindings image: method bitmap, type bitmap, enum values (int32, little-endian)
static const uint16_t INFO_TYPES_OFFSET = (BINDING_METHOD_COUNT + 7) / 8;
static const uint16_t INFO_ENUMS_OFFSET = INFO_TYPES_OFFSET + (BINDING_TYPE_COUNT + 7) / 8;
static const uint16_t BINDING_INFO_SIZE = INFO_ENUMS_OFFSET + 4 * BINDING_ENUM_COUNT;
static_assert(BINDING_INFO_SIZE <= MAX_STRING, "the m4:bindings image is one string value");

inline bool info_method_available(const uint8_t *info, uint16_t method)
{
    if (method < FIRST_BINDING_METHOD) {
        return false;
    }
    const uint16_t index = uint16_t(method - FIRST_BINDING_METHOD);
    return index < BINDING_METHOD_COUNT && (info[index / 8] & (1U << (index % 8))) != 0;
}

inline void info_set_method(uint8_t *info, uint16_t method)
{
    const uint16_t index = uint16_t(method - FIRST_BINDING_METHOD);
    if (method >= FIRST_BINDING_METHOD && index < BINDING_METHOD_COUNT) {
        info[index / 8] |= uint8_t(1U << (index % 8));
    }
}

inline bool info_type_available(const uint8_t *info, uint8_t type_id)
{
    if (type_id == 0 || type_id > BINDING_TYPE_COUNT) {
        return false;
    }
    const uint8_t index = uint8_t(type_id - 1);
    return (info[INFO_TYPES_OFFSET + index / 8] & (1U << (index % 8))) != 0;
}

inline void info_set_type(uint8_t *info, uint8_t type_id)
{
    if (type_id != 0 && type_id <= BINDING_TYPE_COUNT) {
        const uint8_t index = uint8_t(type_id - 1);
        info[INFO_TYPES_OFFSET + index / 8] |= uint8_t(1U << (index % 8));
    }
}

inline int32_t info_enum_value(const uint8_t *info, uint16_t index)
{
    int32_t value = 0;
    if (index < BINDING_ENUM_COUNT) {
        memcpy(&value, &info[INFO_ENUMS_OFFSET + 4 * index], sizeof(value));
    }
    return value;
}

inline void info_set_enum(uint8_t *info, uint16_t index, int32_t value)
{
    if (index < BINDING_ENUM_COUNT) {
        memcpy(&info[INFO_ENUMS_OFFSET + 4 * index], &value, sizeof(value));
    }
}

// "type:row" of a binding (ahrs:get_location), else the system method's name; SRPC
// rows. No printf: the privileged M4 service calls it
inline void binding_method_label(uint16_t method, char *buf, uint8_t size)
{
    const char *type_name = nullptr;
    const char *row_name = method_name(method);
    if (method >= FIRST_BINDING_METHOD && uint16_t(method - FIRST_BINDING_METHOD) < BINDING_METHOD_COUNT) {
        const uint16_t row = binding_method_rows[method - FIRST_BINDING_METHOD];
        if (row < BINDING_ROW_COUNT) {
            type_name = binding_types[binding_rows[row].type - 1].name;
            row_name = binding_rows[row].name;
        }
    }
    uint8_t length = 0;
    const char *parts[] = { type_name, type_name != nullptr ? ":" : nullptr, row_name };
    for (const char *part : parts) {
        for (; part != nullptr && *part != 0 && length + 1 < size; part++) {
            buf[length++] = *part;
        }
    }
    if (size > 0) {
        buf[length] = 0;
    }
}

}  // namespace ScriptingM4
