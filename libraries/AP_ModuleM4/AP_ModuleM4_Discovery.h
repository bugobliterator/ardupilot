#pragma once

/*
  The module files of an M4 session, found on the M7 by the in-process
  runtime's rules (AP_Module Runtime::load_generation()): *.apmod directly in
  @ROMFS/modules and in the writable module directory (/APM/modules; ./modules in
  SITL), a ROMFS file overriding a writable one of the same name, sorted by
  name, and more unique files than MOD_MAX_MOD a load failure. The
  directories are read through a DirectoryLister, so tests feed their own.
 */

#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED

#include <stdint.h>
#include "AP_ModuleM4_Files.h"

namespace ModuleM4 {

static const uint8_t MODULE_PATH_MAX = 96;

extern const char ROMFS_MODULE_DIRECTORY[];
extern const char WRITABLE_MODULE_DIRECTORY[];

struct ModuleFile {
    char name[MODULE_NAME_MAX];   // "airshow.apmod"
    char path[MODULE_PATH_MAX];   // what the M4 opens: module_path_allowed()
    uint32_t size;
    bool romfs;
};

// DirectoryUnavailable: the writable directory exists but could not be read (the PR's
// "module directory unavailable" and "directory read" failures)
enum class DiscoveryError : uint8_t { None, TooMany, NameTooLong, DirectoryUnavailable };

struct DirectoryEntry {
    const char *name;
    bool directory;
    uint32_t size;
};

using DirectoryEntryCallback = void (*)(void *walk, const DirectoryEntry &entry);

struct DirectoryLister {
    void *context;
    // calls entry(walk, ...) for each entry of directory; a directory that does not exist lists
    // nothing. False if it exists and could not be read
    bool (*list)(void *context, const char *directory, DirectoryEntryCallback entry, void *walk);
    // a writable file is ignored for a ROMFS one of the same name; may be null
    void (*duplicate)(void *context, const char *name);
};

// the modules found, sorted by name, in out[0 .. count); 0 with error set when they cannot all load
uint8_t discover_modules(const DirectoryLister &lister, uint8_t max_modules, ModuleFile *out, uint8_t capacity,
                         DiscoveryError &error);

#if AP_MODULE_M4_ENABLED
// AP_Filesystem's directories; a duplicate is reported to the GCS in the PR's words
const DirectoryLister &filesystem_lister();
#endif

}  // namespace ModuleM4

#endif  // AP_MODULE_ENABLED
