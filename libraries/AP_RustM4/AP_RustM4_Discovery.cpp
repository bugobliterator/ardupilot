#include "AP_RustM4_Discovery.h"

#if AP_SCRIPTING_RUST_ENABLED

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <AP_Common/AP_Common.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

namespace RustM4 {

const char ROMFS_MODULE_DIRECTORY[] = "@ROMFS/rust";
#if HAL_OS_FATFS_IO || HAL_OS_LITTLEFS_IO
const char WRITABLE_MODULE_DIRECTORY[] = "/APM/rust";
#else
// the PR's "./rust", spelled as module_path_allowed() takes it
const char WRITABLE_MODULE_DIRECTORY[] = "rust";
#endif

static const char MODULE_SUFFIX[] = ".apmod";

namespace {

struct Walk {
    const DirectoryLister *lister;
    const char *directory;
    bool romfs;
    uint8_t max_modules;
    ModuleFile *out;
    uint8_t count;
    uint8_t romfs_count;
    DiscoveryError error;
};

void add_entry(void *state, const DirectoryEntry &entry)
{
    Walk &walk = *static_cast<Walk *>(state);
    if (walk.error != DiscoveryError::None || entry.directory || entry.name == nullptr) {
        return;
    }
    const size_t suffix_length = sizeof(MODULE_SUFFIX) - 1;
    const size_t length = strlen(entry.name);
    // non-recursive, as the PR: a name with a separator is in a subdirectory
    if (length <= suffix_length || strcmp(entry.name + length - suffix_length, MODULE_SUFFIX) != 0 ||
        strchr(entry.name, '/') != nullptr || strchr(entry.name, '\\') != nullptr) {
        return;
    }
    if (!walk.romfs) {
        for (uint8_t i = 0; i < walk.romfs_count; i++) {
            if (strcmp(walk.out[i].name, entry.name) == 0) {
                if (walk.lister->duplicate != nullptr) {
                    walk.lister->duplicate(walk.lister->context, entry.name);
                }
                return;
            }
        }
    }
    if (length >= MODULE_NAME_MAX) {
        walk.error = DiscoveryError::NameTooLong;
        return;
    }
    if (walk.count >= walk.max_modules) {
        walk.error = DiscoveryError::TooMany;
        return;
    }
    ModuleFile &file = walk.out[walk.count];
    memcpy(file.name, entry.name, length + 1);
    snprintf(file.path, sizeof(file.path), "%s/%s", walk.directory, entry.name);
    file.size = entry.size;
    file.romfs = walk.romfs;
    walk.count++;
}

}  // namespace

uint8_t discover_modules(const DirectoryLister &lister, uint8_t max_modules, ModuleFile *out, uint8_t capacity,
                         DiscoveryError &error)
{
    static_assert(sizeof("@ROMFS/rust/") - 1 + MODULE_NAME_MAX <= MODULE_PATH_MAX, "every module path fits");
    Walk walk {};
    walk.lister = &lister;
    walk.max_modules = max_modules < capacity ? max_modules : capacity;
    walk.out = out;
    walk.error = DiscoveryError::None;

    walk.directory = ROMFS_MODULE_DIRECTORY;
    walk.romfs = true;
    // the PR lists the ROMFS it was built with: no ROMFS directory is no ROMFS module
    IGNORE_RETURN(lister.list(lister.context, ROMFS_MODULE_DIRECTORY, add_entry, &walk));
    walk.romfs_count = walk.count;

    if (walk.error == DiscoveryError::None) {
        walk.directory = WRITABLE_MODULE_DIRECTORY;
        walk.romfs = false;
        if (!lister.list(lister.context, WRITABLE_MODULE_DIRECTORY, add_entry, &walk) &&
            walk.error == DiscoveryError::None) {
            walk.error = DiscoveryError::DirectoryUnavailable;
        }
    }
    error = walk.error;
    if (error != DiscoveryError::None) {
        return 0;
    }
    // a stable order whatever order the directories list in
    for (uint8_t i = 1; i < walk.count; i++) {
        for (uint8_t j = i; j > 0 && strcmp(out[j - 1].name, out[j].name) > 0; j--) {
            const ModuleFile temporary = out[j];
            out[j] = out[j - 1];
            out[j - 1] = temporary;
        }
    }
    return walk.count;
}

}  // namespace RustM4

#if AP_RUST_M4_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>

namespace RustM4 {

namespace {

bool filesystem_list(void *context, const char *directory, DirectoryEntryCallback entry, void *walk)
{
    (void)context;
    auto &fs = AP::FS();
    errno = 0;
    auto *handle = fs.opendir(directory);
    if (handle == nullptr) {
        return errno == ENOENT;
    }
    errno = 0;
    while (const struct dirent *de = fs.readdir(handle)) {
        DirectoryEntry e {};
        e.name = de->d_name;
#if AP_FILESYSTEM_HAVE_DIRENT_DTYPE
        e.directory = de->d_type == DT_DIR;
#endif
        char path[MODULE_PATH_MAX];
        struct stat st {};
        if (snprintf(path, sizeof(path), "%s/%s", directory, de->d_name) < int(sizeof(path)) &&
            fs.stat(path, &st) == 0) {
            e.size = uint32_t(st.st_size);
        }
        entry(walk, e);
        // stat() of a name that is not a module may fail: only readdir()'s errno counts
        errno = 0;
    }
    const bool read_error = errno != 0;
    const bool closed = fs.closedir(handle) == 0;
    return !read_error && closed;
}

void filesystem_duplicate(void *context, const char *name)
{
    (void)context;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Rust: ignored writable %.31s; ROMFS wins", name);
}

}  // namespace

const DirectoryLister &filesystem_lister()
{
    static const DirectoryLister lister { nullptr, filesystem_list, filesystem_duplicate };
    return lister;
}

}  // namespace RustM4

#endif  // AP_RUST_M4_ENABLED

#endif  // AP_SCRIPTING_RUST_ENABLED
