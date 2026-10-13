/*
  logger:write() of lua_bindings.cpp ported to the remote adapter, and the M7 replacement of the
  generated logger:log_file_content() (gen-bindings -r remote_ready_manual[],
  remote_replaced_methods[]). Up to 21 values cross: the self, five strings and 15 fields
  (MAX_VALUES). A dot call (logger.write(...)) arrives with a nil self (ROW_DOT_ACCESS).

  Everything logger:write() takes is checked, and the row built, before msg_fmt_for_name() is
  asked for a message type: that takes a type and M7 memory for good, leaks its copy of the
  strings when the row is too long, and on SITL stops the host thread on what it finds invalid.

  Waits on the host thread (the 900 ms rule): log_write_fmts_sem in msg_fmt_for_name(), and the
  backend's write semaphore in WriteBlock(), each held by the main loop and the logging thread for
  one block, the wait of every vehicle log write; FileContent::sem in log_file_content() and
  file_content_queued(), held by the logger thread for one 64-byte step; and log_file_content()'s
  stat() of the path, one lookup on the filesystem, as the M4's file calls make.
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include "AP_ScriptingM4_Quota.h"
#include "AP_ScriptingM4_Remote.h"

extern const AP_HAL::HAL& hal;

namespace ScriptingM4Remote {

// lua.h's LUA_TBOOLEAN, as the adapter's lua_type() reports true and false
static const int LUA_TYPE_BOOLEAN = 1;

#if HAL_LOGGING_ENABLED
static bool same_string(const char *a, const char *b)
{
    return (a == nullptr) == (b == nullptr) && (a == nullptr || strcmp(a, b) == 0);
}

// M4: whether the logger has a format of this name, and whether it is the one asked for. The
// list is walked as the logger's own readers walk it (log_write_fmt_for_msg_type(),
// AP_Logger_Backend::Write()), without its semaphore: entries are published complete by one
// pointer store and never freed. msg_fmt_for_name() returns the first of the name, which this
// finds too
enum class LoggerFormat : uint8_t { Absent, Same, Differs };
static LoggerFormat logger_format_of(const AP_Logger &logger, const char *name, const char *labels,
                                     const char *units, const char *mults, const char *fmt)
{
    for (const AP_Logger::log_write_fmt *f = logger.log_write_fmts; f != nullptr; f = f->next) {
        if (strcmp(f->name, name) == 0) {
            const bool same = same_string(f->labels, labels) && same_string(f->fmt, fmt) &&
                              same_string(f->units, units) && same_string(f->mults, mults);
            return same ? LoggerFormat::Same : LoggerFormat::Differs;
        }
    }
    return LoggerFormat::Absent;
}

// message types logger:write has taken this boot (LOGGER_SCRIPT_TYPES_MAX); kept across sessions,
// as the types are
static uint8_t script_log_types;

// logger:write(name, labels, format, [units, multipliers,] values...): AP_Logger_Write()
int AP_Logger_Write(lua_State *L) {
    AP_Logger * AP_logger = AP_Logger::get_singleton();
    if (AP_logger == nullptr) {
        return luaL_argerror(L, 1, "logger not supported on this firmware");
    }

    // M4: no fix_dot_access_never_add_another_call(): a dot call arrives with a nil self

    // check we have at least 5 arguments passed in
    const size_t args = lua_gettop(L);
    if (args < 5) {
        return luaL_argerror(L, int(args), "too few arguments");
    }

    const char * name = luaL_checkstring(L, 2);
    const char * labels = luaL_checkstring(L, 3);
    const char * fmt = luaL_checkstring(L, 4);

    // cheack the name, labels and format are not too long
    if (strlen(name) >= LS_NAME_SIZE) {
        return luaL_error(L, "Name must be 4 or less chars long");
    }
    // M4: a name of letters, digits and '_'
    if (!logger_name_valid(name)) {
        return luaL_error(L, "Name must be letters, digits or _");
    }
    size_t labels_length = strlen(labels);
    if (labels_length >= (LS_LABELS_SIZE - 7)) { // need 7 chars to add 'TimeUS,'
        return luaL_error(L, "labels must be less than 58 chars long");
    }
    // Count the number of commas
    size_t commas = 1;
    for (size_t i=0; i<labels_length; i++) {
        if (labels[i] == ',') {
            commas++;
        }
    }

    size_t fmt_length = strlen(fmt);
    if (fmt_length >= (LS_FORMAT_SIZE - 1)) { // need 1 char to add timestamp
        return luaL_error(L, "format must be less than 15 chars long");
    }

    // check the number of arguments matches the number of values in the label
    if (fmt_length != commas) {
        return luaL_argerror(L, int(args), "label does not match format");
    }

    // M4: every format character is one this function writes
    const char *unsupported = logger_format_unsupported(fmt);
    if (unsupported != nullptr) {
        return luaL_error(L, "%c unsupported format", *unsupported);
    }

    bool have_units = false;
    if (args - 6 == fmt_length) {
        // check if there are enough arguments for units and multipliers
        have_units = true;
    } else if (args - 4 != fmt_length) {
        // check the number of arguments matches the length of the format string
        return luaL_argerror(L, int(args), "format does not match No. of arguments");
    }

    // prepend timestamp to format and labels
    char label_cat[LS_LABELS_SIZE];
    char fmt_cat[LS_FORMAT_SIZE];
    snprintf(label_cat, sizeof(label_cat), "TimeUS,%s", labels);
    snprintf(fmt_cat, sizeof(fmt_cat), "Q%s", fmt);

    // M4: labels SITL's logger takes (no empty or repeated label)
    if (!logger_labels_valid(label_cat)) {
        return luaL_error(L, "labels must be unique and not empty");
    }

    // M4: the row's length, before msg_fmt_for_name(), which leaks its copy of the strings on a
    // row longer than LOG_PACKET_MAX_LEN (in-process this is checked after it)
    // work out how long the block will be
    int16_t msg_len = AP_logger->Write_calc_msg_len(fmt_cat);
    if (msg_len == -1) {
        return luaL_argerror(L, int(args), "unknown format");
    }

    uint8_t field_start = 4;
    char units_cat[LS_FORMAT_SIZE];
    char multipliers_cat[LS_FORMAT_SIZE];
    if (have_units) {
        // read in units and multipliers strings
        field_start += 2;
        const char * units = luaL_checkstring(L, 5);
        const char * multipliers = luaL_checkstring(L, 6);

        if (fmt_length != strlen(units)) {
            return luaL_error(L, "units must be same length as format");
        }
        if (fmt_length != strlen(multipliers)) {
            return luaL_error(L, "multipliers must be same length as format");
        }
        // M4: units and multipliers the logger has
        const char *unknown = logger_ids_unsupported(units, LOGGER_UNIT_IDS);
        if (unknown != nullptr) {
            return luaL_error(L, "%c unsupported unit", *unknown);
        }
        unknown = logger_ids_unsupported(multipliers, LOGGER_MULTIPLIER_IDS);
        if (unknown != nullptr) {
            return luaL_error(L, "%c unsupported multiplier", *unknown);
        }

        // prepend timestamp to units and multipliers
        snprintf(units_cat, sizeof(units_cat), "s%s", units);
        snprintf(multipliers_cat, sizeof(multipliers_cat), "F%s", multipliers);
    }

    // M4: a name keeps the format of its first row, and a new name needs one of the script's
    // message types
    const LoggerFormat existing = logger_format_of(*AP_logger, name, label_cat, have_units ? units_cat : nullptr,
                                                   have_units ? multipliers_cat : nullptr, fmt_cat);
    if (existing == LoggerFormat::Differs) {
        return luaL_argerror(L, int(args), "format differs from an earlier write");
    }
    if (existing == LoggerFormat::Absent && script_log_types >= LOGGER_SCRIPT_TYPES_MAX) {
        return luaL_argerror(L, int(args), "too many log message types");
    }

    // M4: no luaL_Buffer static_assert (the adapter has none). The writes below stay in the
    // buffer: offset sums the sizes Write_calc_msg_len() summed, at most LOG_PACKET_MAX_LEN
    char buffer[LOG_PACKET_MAX_LEN]; // constant buffer size optimizes better

    // add logging headers (M4: the message type once the logger has given it, below)
    uint8_t offset = 0;
    buffer[offset++] = HEAD_BYTE1;
    buffer[offset++] = HEAD_BYTE2;
    offset++;

    // timestamp is always first value
    const uint64_t now = AP_HAL::micros64();
    memcpy(&buffer[offset], &now, sizeof(uint64_t));
    offset += sizeof(uint64_t);

    // M4: the values are read before msg_fmt_for_name(), so a bad one takes nothing
    for (uint8_t i=field_start; i<=args-1; i++) {
        uint8_t charlen = 0;
        uint8_t index = have_units ? i-5 : i-3;
        uint8_t arg_index = i + 1;
        switch(fmt_cat[index]) {
            // logger variable types not available to scripting
            // 'd': double
            // 'q': int64_t
            // 'a': int16_t[32]
            case 'b': { // int8_t
                int8_t tmp = get_int8_t(L, arg_index);
                memcpy(&buffer[offset], &tmp, sizeof(int8_t));
                offset += sizeof(int8_t);
                break;
            }
            case 'h': // int16_t
            case 'c': { // int16_t * 100
                int16_t tmp = get_int16_t(L, arg_index);
                memcpy(&buffer[offset], &tmp, sizeof(int16_t));
                offset += sizeof(int16_t);
                break;
            }
            case 'H': // uint16_t
            case 'C': { // uint16_t * 100
                uint16_t tmp = get_uint16_t(L, arg_index);
                memcpy(&buffer[offset], &tmp, sizeof(uint16_t));
                offset += sizeof(uint16_t);
                break;
            }
            case 'i': // int32_t
            case 'L': // int32_t (lat/long)
            case 'e': { // int32_t * 100
                const lua_Integer tmp1 = luaL_checkinteger(L, arg_index);
                const int32_t tmp = tmp1;
                memcpy(&buffer[offset], &tmp, sizeof(int32_t));
                offset += sizeof(int32_t);
                break;
            }
            case 'f': { // float
                const lua_Number tmp1 = luaL_checknumber(L, arg_index);
                const float tmp = tmp1;
                memcpy(&buffer[offset], &tmp, sizeof(float));
                offset += sizeof(float);
                break;
            }
            case 'n': { // char[4]
                charlen = 4;
                break;
            }
            case 'M': // uint8_t (flight mode)
            case 'B': { // uint8_t
                uint8_t tmp;
                if (lua_type(L, arg_index) == LUA_TYPE_BOOLEAN) { // Also allow boolean (M4: lua_isboolean())
                    tmp = static_cast<uint8_t>(lua_toboolean(L, arg_index));
                } else {
                    tmp = get_uint8_t(L, arg_index);
                }
                memcpy(&buffer[offset], &tmp, sizeof(uint8_t));
                offset += sizeof(uint8_t);
                break;
            }
            case 'I': // uint32_t
            case 'E': { // uint32_t * 100
                uint32_t tmp = coerce_to_uint32_t(L, arg_index);
                memcpy(&buffer[offset], &tmp, sizeof(uint32_t));
                offset += sizeof(uint32_t);
                break;
            }
            case 'Q': { // uint64_t
                uint64_t tmp = coerce_to_uint64_t(L, arg_index);
                memcpy(&buffer[offset], &tmp, sizeof(uint64_t));
                offset += sizeof(uint64_t);
                break;
            }
            case 'N': { // char[16]
                charlen = 16;
                break;
            }
            case 'Z': { // char[64]
                charlen = 64;
                break;
            }
            default: {
                return luaL_error(L, "%c unsupported format", fmt_cat[index]);
            }
        }
        if (charlen != 0) {
            size_t slen;
            const char *tmp = luaL_checklstring(L, arg_index, &slen);
            if (slen > charlen) {
                return luaL_error(L, "arg %d too long for %c format", arg_index, fmt_cat[index]);
            }
            memcpy(&buffer[offset], tmp, slen);
            memset(&buffer[offset+slen], 0, charlen-slen);
            offset += charlen;
        }
    }

    // M4: the Log quota, once the row is built and before the logger takes anything; over it the
    // row is dropped (the method is void)
    if (!remote_quota_allow(L, ScriptingM4::QuotaClass::Log)) {
        return 0;
    }

    struct AP_Logger::log_write_fmt *f;
    if (!have_units) {
        // ask for a message type (will duplicate incoming strings if necessary)
        f = AP_logger->msg_fmt_for_name(name, label_cat, nullptr, nullptr, fmt_cat, true, true);
    } else {
        // ask for a message type (will duplicate incoming strings if necessary)
        f = AP_logger->msg_fmt_for_name(name, label_cat, units_cat, multipliers_cat, fmt_cat, true, true);
    }

    if (f == nullptr) {
        // unable to map name to a messagetype; could be out of
        // msgtypes, could be out of slots, ...
        return luaL_argerror(L, int(args), "could not map message type");
    }
    if (existing == LoggerFormat::Absent) {
        script_log_types++;
    }
    buffer[2] = f->msg_type;

    AP_logger->Safe_Write_Emit_FMT(f);

    AP_logger->WriteBlock(buffer, msg_len);

    return 0;
}
#endif // HAL_LOGGING_ENABLED

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
// generated (lua_remote_dispatch.cpp)
AP_Logger *check_AP_Logger(lua_State *L);

// AP_Logger_File's directory: the board's, or the one the HAL was given
static const char *logger_directory()
{
    const char *custom = hal.util->get_custom_log_directory();
    return custom != nullptr ? custom : HAL_BOARD_LOG_DIRECTORY;
}

// logger:log_file_content(path): the generated method, with the path checked as the M4's file
// calls check it and the logger's queue bounded (file_content_allowed()). A refused call is
// dropped, as the void method has no result. The FileLog quota is the server's (remote_quotas[])
int lua_m4_AP_Logger_log_file_content(lua_State *L)
{
    binding_argcheck(L, 2);
    AP_Logger *ud = check_AP_Logger(L);
    size_t length = 0;
    const char *path = luaL_checklstring(L, 2, &length);
    if (!file_content_allowed(path, length, ud->file_content_queued(FILE_CONTENT_QUEUE_MAX))) {
        return 0;
    }
    // M4: never the logger's own directory, by any name AP_Filesystem opens it under (the log
    // being written does not end), and only a regular file or an "@" file (which stat() gives no
    // type): a device does not end either
    if (path_in_log_directory(path, logger_directory())) {
        return 0;
    }
    struct stat st;
    if (AP::FS().stat(path, &st) != 0 || ((st.st_mode & S_IFMT) != 0 && !S_ISREG(st.st_mode))) {
        return 0;
    }
    ud->log_file_content(path);
    return 0;
}
#endif  // HAL_LOGGER_FILE_CONTENTS_ENABLED

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
