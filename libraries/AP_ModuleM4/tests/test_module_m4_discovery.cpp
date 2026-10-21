#include <AP_gtest.h>
#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED
#include <map>
#include <string>
#include <vector>
#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem_config.h>
#include <AP_ModuleM4/AP_ModuleM4_Discovery.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace ModuleM4;

// directories as maps of name -> (directory, size); a missing key is a missing directory
class FakeLister {
public:
    struct Entry {
        std::string name;
        bool directory;
        uint32_t size;
    };

    void add(const char *directory, const char *name, uint32_t size = 100, bool is_directory = false)
    {
        directories[directory].push_back(Entry { name, is_directory, size });
    }

    DirectoryLister lister() { return DirectoryLister { this, list, duplicate }; }

    std::map<std::string, std::vector<Entry>> directories;
    std::vector<std::string> unreadable;
    std::vector<std::string> duplicates;

private:
    static bool list(void *context, const char *directory, DirectoryEntryCallback entry, void *walk)
    {
        FakeLister &self = *static_cast<FakeLister *>(context);
        for (const std::string &bad : self.unreadable) {
            if (bad == directory) {
                return false;
            }
        }
        const auto found = self.directories.find(directory);
        if (found == self.directories.end()) {
            return true;
        }
        for (const Entry &e : found->second) {
            entry(walk, DirectoryEntry { e.name.c_str(), e.directory, e.size });
        }
        return true;
    }

    static void duplicate(void *context, const char *name)
    {
        static_cast<FakeLister *>(context)->duplicates.push_back(name);
    }
};

static std::string writable(const char *name)
{
    return std::string(WRITABLE_MODULE_DIRECTORY) + "/" + name;
}

TEST(ModuleM4Discovery, FindsOnlyApmodFiles)
{
    FakeLister fake;
    fake.add(WRITABLE_MODULE_DIRECTORY, "a.apmod", 1234);
    fake.add(WRITABLE_MODULE_DIRECTORY, "b.txt");
    fake.add(WRITABLE_MODULE_DIRECTORY, "sub", 0, true);
    fake.add(WRITABLE_MODULE_DIRECTORY, "dir.apmod", 0, true);
    fake.add(WRITABLE_MODULE_DIRECTORY, ".apmod");
    fake.add(WRITABLE_MODULE_DIRECTORY, "c.apmod.bak");
    ModuleFile files[8];
    DiscoveryError error = DiscoveryError::TooMany;
    ASSERT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 1U);
    EXPECT_EQ(error, DiscoveryError::None);
    EXPECT_STREQ(files[0].name, "a.apmod");
    EXPECT_EQ(files[0].path, writable("a.apmod"));
    EXPECT_EQ(files[0].size, 1234U);
    EXPECT_FALSE(files[0].romfs);
    EXPECT_TRUE(module_path_allowed(files[0].path));
}

TEST(ModuleM4Discovery, DirectoriesAreAP_Modules)
{
    // AP_Module Runtime: ROMFS modules/, then /APM/modules (./modules in SITL)
    EXPECT_STREQ(ROMFS_MODULE_DIRECTORY, "@ROMFS/modules");
#if HAL_OS_FATFS_IO || HAL_OS_LITTLEFS_IO
    EXPECT_STREQ(WRITABLE_MODULE_DIRECTORY, "/APM/modules");
#else
    EXPECT_STREQ(WRITABLE_MODULE_DIRECTORY, "modules");
#endif
    FakeLister fake;
    fake.add("@ROMFS/rust", "old.apmod");
    fake.add("rust", "old.apmod");
    fake.add("/APM/rust", "old.apmod");
    fake.add(WRITABLE_MODULE_DIRECTORY, "new.apmod");
    ModuleFile files[8];
    DiscoveryError error;
    ASSERT_EQ(discover_modules(fake.lister(), 8, files, 8, error), 1U);
    EXPECT_STREQ(files[0].name, "new.apmod");
    EXPECT_TRUE(module_path_allowed(files[0].path));
}

TEST(ModuleM4Discovery, RomfsWinsADuplicate)
{
    FakeLister fake;
    fake.add(ROMFS_MODULE_DIRECTORY, "airshow.apmod", 10);
    fake.add(WRITABLE_MODULE_DIRECTORY, "airshow.apmod", 20);
    ModuleFile files[8];
    DiscoveryError error;
    ASSERT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 1U);
    EXPECT_EQ(error, DiscoveryError::None);
    EXPECT_TRUE(files[0].romfs);
    EXPECT_EQ(files[0].size, 10U);
    EXPECT_STREQ(files[0].path, "@ROMFS/modules/airshow.apmod");
    EXPECT_TRUE(module_path_allowed(files[0].path));
    ASSERT_EQ(fake.duplicates.size(), 1U);
    EXPECT_EQ(fake.duplicates[0], "airshow.apmod");
}

TEST(ModuleM4Discovery, SortedByName)
{
    FakeLister fake;
    fake.add(WRITABLE_MODULE_DIRECTORY, "zeta.apmod");
    fake.add(ROMFS_MODULE_DIRECTORY, "mid.apmod");
    fake.add(WRITABLE_MODULE_DIRECTORY, "alpha.apmod");
    ModuleFile files[8];
    DiscoveryError error;
    ASSERT_EQ(discover_modules(fake.lister(), 8, files, 8, error), 3U);
    EXPECT_STREQ(files[0].name, "alpha.apmod");
    EXPECT_STREQ(files[1].name, "mid.apmod");
    EXPECT_TRUE(files[1].romfs);
    EXPECT_STREQ(files[2].name, "zeta.apmod");
    EXPECT_EQ(files[2].path, writable("zeta.apmod"));
}

TEST(ModuleM4Discovery, TooManyIsAnError)
{
    FakeLister fake;
    fake.add(WRITABLE_MODULE_DIRECTORY, "a.apmod");
    fake.add(WRITABLE_MODULE_DIRECTORY, "b.apmod");
    fake.add(WRITABLE_MODULE_DIRECTORY, "c.apmod");
    ModuleFile files[8];
    DiscoveryError error;
    EXPECT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 0U);
    EXPECT_EQ(error, DiscoveryError::TooMany);
    // duplicates do not count: they are not unique files
    FakeLister two;
    two.add(ROMFS_MODULE_DIRECTORY, "a.apmod");
    two.add(WRITABLE_MODULE_DIRECTORY, "a.apmod");
    two.add(WRITABLE_MODULE_DIRECTORY, "b.apmod");
    EXPECT_EQ(discover_modules(two.lister(), 2, files, 8, error), 2U);
    EXPECT_EQ(error, DiscoveryError::None);
    // the caller's capacity bounds MOD_MAX_MOD
    EXPECT_EQ(discover_modules(fake.lister(), 8, files, 2, error), 0U);
    EXPECT_EQ(error, DiscoveryError::TooMany);
}

TEST(ModuleM4Discovery, EmptyIsNotAnError)
{
    FakeLister fake;
    ModuleFile files[8];
    DiscoveryError error = DiscoveryError::TooMany;
    EXPECT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 0U);
    EXPECT_EQ(error, DiscoveryError::None);
    fake.directories[WRITABLE_MODULE_DIRECTORY];
    EXPECT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 0U);
    EXPECT_EQ(error, DiscoveryError::None);
}

TEST(ModuleM4Discovery, LongNameAndUnreadableDirectoryAreErrors)
{
    ModuleFile files[8];
    DiscoveryError error;
    FakeLister fake;
    const std::string longest = std::string(MODULE_NAME_MAX - 1 - 6, 'n') + ".apmod";
    fake.add(WRITABLE_MODULE_DIRECTORY, longest.c_str());
    ASSERT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 1U);
    EXPECT_TRUE(module_path_allowed(files[0].path));
    const std::string too_long = "n" + longest;
    fake.add(WRITABLE_MODULE_DIRECTORY, too_long.c_str());
    EXPECT_EQ(discover_modules(fake.lister(), 2, files, 8, error), 0U);
    EXPECT_EQ(error, DiscoveryError::NameTooLong);
    FakeLister unreadable;
    unreadable.add(ROMFS_MODULE_DIRECTORY, "a.apmod");
    unreadable.unreadable.push_back(WRITABLE_MODULE_DIRECTORY);
    EXPECT_EQ(discover_modules(unreadable.lister(), 2, files, 8, error), 0U);
    EXPECT_EQ(error, DiscoveryError::DirectoryUnavailable);
}

#endif  // AP_MODULE_ENABLED

AP_GTEST_MAIN()
