#include <AP_gtest.h>
#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED
#include <stdlib.h>
#include <string.h>
#include <map>
#include <string>
#include <vector>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_Module/Bindings.h>
#include <AP_ModuleM4_Placement.h>
#include "module_host.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

using namespace ModuleM4;
using AP_Module::Image;

namespace {

static const uint64_t REMOTE_HASH = 0x1122334455667788ULL;
static const char FILE_NAME[] = "m4_test.apmod";

static Image::Compatibility compatibility()
{
    return { 62U, 1, 2U, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16} };
}

// as the loader parses at MOD_STACK_MAX 32768 with the x86-64 margin
static Image::Compatibility parse_compatibility()
{
    Image::Compatibility c = compatibility();
    c.stack_limit = 32768U - 16384U;
    return c;
}

// an id the M4 does not answer itself
static uint32_t remote_id()
{
    uint32_t id = 1;
    while (AP_Module::binding_is_local(id)) {
        id++;
    }
    return id;
}

/*
  An APMOD06 split container for the host's machine (test_rust_image.cpp's SplitContainer:
  4096 B of code, two fixups, RW and BSS), with an import table of two hashes added the way its
  ImportContainer adds one: a local hash of the M4's catalogue and a remote one.
 */
struct SplitImportContainer {
    static constexpr uint32_t header_size = 84;
    static constexpr uint32_t code_size = 4096;
    static constexpr uint32_t file_size = code_size + 2 * sizeof(uintptr_t) + 8;
    static constexpr uint32_t memory_size = file_size + 16;
    static constexpr uint32_t fixup_count = 2;
    static constexpr uint32_t import_count = 2;
    static constexpr uint32_t local_slot = code_size + 2 * sizeof(uintptr_t);
    static constexpr uint32_t remote_slot = local_slot + 4;
    static constexpr char names[] = "local.api\0remote.api";
    std::vector<uint8_t> bytes;

    void put(uint32_t offset, uint32_t value)
    {
        for (uint8_t i = 0; i < 4; i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void put_pointer(uint32_t offset, uintptr_t value)
    {
        for (uint8_t i = 0; i < sizeof(uintptr_t); i++) {
            bytes[offset + i] = uint8_t(value >> (8 * i));
        }
    }

    void checksum()
    {
        put(80, 0);
        put(80, crc32_small(UINT32_MAX, bytes.data(), bytes.size()) ^ UINT32_MAX);
    }

    SplitImportContainer() : bytes(header_size + file_size + fixup_count * 4 + import_count * 16 + sizeof(names))
    {
        memcpy(bytes.data(), "APMOD06", 8);
        put(8, 6);
        const auto expected = compatibility();
        put(12, expected.machine);
        put(16, sizeof(uintptr_t));
        put(20, sizeof(uintptr_t) | sizeof(uintptr_t) << 16);
        put(24, file_size);
        put(28, memory_size);
        put(32, code_size);
        put(36, 1);
        put(40, fixup_count);
        put(44, expected.abi_version);
        put(48, expected.cpu_profile);
        put(52, 4096);
        put(56, 32768);
        put(60, import_count);
        memcpy(bytes.data() + 64, expected.fingerprint, sizeof(expected.fingerprint));
        bytes[header_size] = 0xA5;
        put_pointer(header_size + sizeof(uintptr_t), 17);
        put_pointer(header_size + code_size, 17);
        put(header_size + local_slot, UINT32_MAX);
        put(header_size + remote_slot, UINT32_MAX);
        const uint32_t fixups = header_size + file_size;
        put(fixups, sizeof(uintptr_t));
        put(fixups + 4, code_size);
        const uint32_t records = fixups + fixup_count * 4;
        const uint64_t local_hash = AP_Module::M4_LOCAL_IMPORTS[0].hash;
        put(records, uint32_t(local_hash));
        put(records + 4, uint32_t(local_hash >> 32));
        put(records + 8, local_slot);
        put(records + 12, 0);
        put(records + 16, uint32_t(REMOTE_HASH));
        put(records + 20, uint32_t(REMOTE_HASH >> 32));
        put(records + 24, remote_slot);
        put(records + 28, 10);
        memcpy(bytes.data() + records + 32, names, sizeof(names));
        checksum();
    }
};
constexpr char SplitImportContainer::names[];

static uint32_t zlib_crc(const uint8_t *data, size_t length)
{
    return crc32_small(UINT32_MAX, data, length) ^ UINT32_MAX;
}

// the M7 as the loader sees it through the ModuleLink: Config, ListModules, the file calls,
// Resolve and the ModuleState notifies; every call is answered at once
struct FakeM7 {
    std::vector<uint8_t> served;   // the bytes FsRead serves
    uint32_t listed_size;
    uint32_t crc_offset = 0;       // added to the CRC FsClose reports
    uint8_t list_count = 1;
    std::map<uint64_t, uint32_t> ids;
    ConfigReply config {};
    uint32_t offset = 0;
    std::vector<std::string> events;
    std::vector<ModuleStateNotify> states;

    explicit FakeM7(const std::vector<uint8_t> &file) : served(file), listed_size(uint32_t(file.size()))
    {
        ids[AP_Module::M4_LOCAL_IMPORTS[0].hash] = AP_Module::M4_LOCAL_IMPORTS[0].id;
        ids[REMOTE_HASH] = remote_id();
        config.heap_size = 32768;
        config.code_size = 131072;
        config.stack_size = 32768;
        config.max_modules = 1;
    }

    static uint16_t reply_header(ModuleLink &link, SystemMethod method, int32_t result, uint16_t length)
    {
        SystemReplyHeader reply {};
        fill_header(reply.header, MsgType::SystemReply, length, 0);
        reply.method = uint16_t(method);
        reply.transport = uint8_t(ReplyStatus::Ok);
        reply.result = result;
        memcpy(link.reply, &reply, sizeof(reply));
        return length;
    }

    void serve(ModuleLink &link)
    {
        if (link.state.load() != ModuleLink::Posted) {
            return;
        }
        SystemCallHeader call;
        memcpy(&call, link.request, sizeof(call));
        ASSERT_EQ(call.header.type, uint8_t(MsgType::SystemCall));
        ASSERT_EQ(call.header.length, link.request_len);
        int32_t result = 0;
        switch (SystemMethod(call.method)) {
        case SystemMethod::ListModules: {
            ListModulesReply reply {};
            reply.count = list_count;
            strcpy(reply.modules[0].name, FILE_NAME);
            strcpy(reply.modules[0].path, "/APM/rust/m4_test.apmod");
            reply.modules[0].size = listed_size;
            memcpy(link.reply, &reply, sizeof(reply));
            result = reply_header(link, SystemMethod::ListModules, list_count, sizeof(reply));
            events.push_back("list");
            break;
        }
        case SystemMethod::FsOpen: {
            FsOpenRequest open;
            memcpy(&open, link.request, sizeof(open));
            EXPECT_STREQ(open.path, "/APM/rust/m4_test.apmod");
            result = reply_header(link, SystemMethod::FsOpen, 3, sizeof(SystemReplyHeader));
            break;
        }
        case SystemMethod::FsRead: {
            FsReadRequest read;
            memcpy(&read, link.request, sizeof(read));
            EXPECT_EQ(read.fd, 3);
            EXPECT_LE(read.count, FS_READ_MAX);
            const uint32_t count = std::min<uint32_t>(read.count, uint32_t(served.size()) - offset);
            memcpy(link.reply + sizeof(SystemReplyHeader), served.data() + offset, count);
            offset += count;
            result = reply_header(link, SystemMethod::FsRead, int32_t(count), uint16_t(sizeof(SystemReplyHeader) + count));
            break;
        }
        case SystemMethod::FsClose: {
            FsCloseReply close {};
            close.crc = zlib_crc(served.data(), offset) + crc_offset;
            close.served = offset;
            memcpy(link.reply, &close, sizeof(close));
            result = reply_header(link, SystemMethod::FsClose, 0, sizeof(close));
            events.push_back("close");
            break;
        }
        case SystemMethod::Resolve: {
            uint16_t count;
            memcpy(&count, link.request + offsetof(ResolveRequest, count), sizeof(count));
            EXPECT_EQ(link.request_len, offsetof(ResolveRequest, hashes) + count * sizeof(uint64_t));
            ResolveReply reply {};
            reply.count = count;
            for (uint16_t i = 0; i < count; i++) {
                uint64_t hash;
                memcpy(&hash, link.request + offsetof(ResolveRequest, hashes) + i * sizeof(hash), sizeof(hash));
                const auto found = ids.find(hash);
                reply.ids[i] = found == ids.end() ? UINT32_MAX : found->second;
            }
            memcpy(link.reply, &reply, sizeof(reply));
            result = reply_header(link, SystemMethod::Resolve, 0,
                                  uint16_t(offsetof(ResolveReply, ids) + count * sizeof(uint32_t)));
            events.push_back("resolve");
            break;
        }
        case SystemMethod::ModuleState: {
            EXPECT_NE(call.flags & CALL_FLAG_NOTIFY, 0);
            ModuleStateNotify notify;
            ASSERT_EQ(link.request_len, sizeof(notify));
            memcpy(&notify, link.request, sizeof(notify));
            states.push_back(notify);
            events.push_back("state:" + std::to_string(notify.state));
            break;
        }
        default:
            ADD_FAILURE() << "unexpected method " << call.method;
            result = -ENOSYS;
            break;
        }
        link.result = result;
        link.state.store(ModuleLink::Done);
    }
};

struct StartRecord {
    bool started = false;
    LoadLayout layout {};
    uint32_t entry = 0;
    uint32_t rw_base = 0;
    ModuleRanges ranges {};
};

/*
  The loader against FakeM7, its module area a buffer that stands for the M4's (the top of the
  area at the M4's address, aligned as there), with module_port_start replaced by a recorder.
 */
struct LoaderRig {
    static constexpr uint32_t buffer_size = AP_MODULE_M4_MODULE_AREA_SIZE;
    uint8_t *buffer = nullptr;
    ModuleLink link {};
    FakeM7 m7;
    ModuleHost host;
    ModuleHostPlatform platform {};
    StartRecord start;
    std::map<uint32_t, std::string> names;
    std::vector<ModuleHost::Step> steps;
    uint32_t unpublished = 0;

    explicit LoaderRig(const std::vector<uint8_t> &file, uint32_t area_size = AP_MODULE_M4_MODULE_AREA_SIZE) : m7(file)
    {
        EXPECT_EQ(posix_memalign(reinterpret_cast<void **>(&buffer), 128U * 1024U, buffer_size), 0);
        memset(buffer, 0xCC, buffer_size);
        const uint32_t top = AP_MODULE_M4_MODULE_AREA_BASE + AP_MODULE_M4_MODULE_AREA_SIZE;
        platform.link = &link;
        platform.area = buffer + (buffer_size - area_size);
        platform.area_base = top - area_size;
        platform.area_size = area_size;
        platform.compatibility = compatibility();
        platform.stack_margin = 16384;
        platform.context = this;
        platform.config = [](void *context, ConfigReply &reply) {
            reply = static_cast<LoaderRig *>(context)->m7.config;
            return true;
        };
        platform.set_binding_name = [](void *context, uint32_t id, const char *name) {
            static_cast<LoaderRig *>(context)->names[id] = name;
        };
        platform.unpublish = [](void *context) {
            static_cast<LoaderRig *>(context)->unpublished++;
        };
        platform.publish = nullptr;
        platform.start = [](void *context, ModuleContext &module, const LoadLayout &layout, uint32_t entry,
                            uint32_t rw_base) {
            auto &rig = *static_cast<LoaderRig *>(context);
            rig.start = { true, layout, entry, rw_base, module.ranges };
            rig.m7.events.push_back("start");
            module.state.store(uint8_t(ModuleStateKind::Started));
        };
        host.init(platform);
    }

    ~LoaderRig()
    {
        free(buffer);
    }

    // the service's step, then the loader's, until the loader stops moving
    void run()
    {
        steps.push_back(host.state());
        for (uint32_t i = 0; i < 1000; i++) {
            m7.serve(link);
            host.step();
            if (host.state() != steps.back()) {
                steps.push_back(host.state());
            }
        }
        m7.serve(link);   // the Failed notify
    }

    uint8_t *at(uint32_t address) const
    {
        return platform.area + (address - platform.area_base);
    }
};

static bool started(const LoaderRig &rig)
{
    return rig.start.started;
}

}  // namespace

TEST(ModuleM4ModuleHost, SplitContainerLoadsToStart)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.run();
    using S = ModuleHost::Step;
    const std::vector<S> expected { S::Config, S::List, S::Open, S::Read, S::Parse, S::Resolve, S::Place, S::Load,
                                    S::Checksum, S::Publish, S::Start, S::Running };
    EXPECT_EQ(rig.steps, expected);
    ASSERT_TRUE(started(rig)) << rig.host.failure();
    EXPECT_EQ(rig.unpublished, 1U);

    // RX top-aligned in the code window, with the PR's contiguous x86-64 image RW right after it
    const LoadLayout &layout = rig.start.layout;
    const uint32_t rx_base = layout.window.rx_base;
    EXPECT_EQ(rig.start.rw_base, rx_base + SplitImportContainer::code_size);
    EXPECT_EQ(rig.start.entry, rx_base + 1U);
    EXPECT_EQ(rig.start.ranges.rx.base, uintptr_t(rig.at(rx_base)));
    EXPECT_EQ(rig.start.ranges.rx.size, SplitImportContainer::code_size);
    EXPECT_EQ(rig.start.ranges.rw.base, uintptr_t(rig.at(rx_base + SplitImportContainer::code_size)));
    EXPECT_EQ(rig.start.ranges.rw.size, SplitImportContainer::memory_size - SplitImportContainer::code_size);
    EXPECT_EQ(rig.start.ranges.stack.base, uintptr_t(rig.at(layout.stack_base)));
    EXPECT_EQ(rig.start.ranges.heap.size, 32768U);

    // the same bytes as Image::load_split run directly; the fixup slots hold this placement's addresses
    Image image;
    ASSERT_TRUE(image.parse(container.bytes.data(), container.bytes.size(), 131072, parse_compatibility()));
    uint8_t *direct = nullptr;
    ASSERT_EQ(posix_memalign(reinterpret_cast<void **>(&direct), 4096, 8192), 0);
    struct Ids {
        static uint32_t resolve(void *, uint64_t hash)
        {
            return hash == REMOTE_HASH ? remote_id() : AP_Module::M4_LOCAL_IMPORTS[0].id;
        }
    };
    ASSERT_TRUE(image.load_split(direct, image.code_size(), direct + image.code_size(), image.rw_memory_size(),
                                 Ids::resolve, nullptr));
    const uint8_t *rx = rig.at(rx_base);
    uintptr_t slot, direct_slot;
    memcpy(&slot, rx + sizeof(uintptr_t), sizeof(slot));
    memcpy(&direct_slot, direct + sizeof(uintptr_t), sizeof(direct_slot));
    EXPECT_EQ(slot, uintptr_t(rx) + 17U);
    EXPECT_EQ(direct_slot, uintptr_t(direct) + 17U);
    EXPECT_EQ(memcmp(rx, direct, sizeof(uintptr_t)), 0);
    EXPECT_EQ(memcmp(rx + 2 * sizeof(uintptr_t), direct + 2 * sizeof(uintptr_t),
                     SplitImportContainer::code_size - 2 * sizeof(uintptr_t)), 0);
    uint32_t local_id, remote;
    memcpy(&local_id, rx + SplitImportContainer::local_slot, sizeof(local_id));
    memcpy(&remote, rx + SplitImportContainer::remote_slot, sizeof(remote));
    EXPECT_EQ(local_id, AP_Module::M4_LOCAL_IMPORTS[0].id);
    EXPECT_EQ(remote, remote_id());
    free(direct);

    // nothing stale above the image in its window
    const uint32_t image_end = rx_base + SplitImportContainer::memory_size;
    const uint32_t window_top = layout.window.base + layout.window.size;
    ASSERT_LT(image_end, window_top);
    for (uint32_t address = image_end; address < window_top; address++) {
        ASSERT_EQ(*rig.at(address), 0) << "RX tail byte at 0x" << std::hex << address;
    }

    // the M4's checksum of the canonical file is the CRC of the bytes the M7 served
    EXPECT_EQ(image.checksum(), zlib_crc(container.bytes.data(), container.bytes.size()));
    ASSERT_EQ(rig.m7.states.size(), 2U);
    EXPECT_EQ(rig.m7.states[0].state, uint8_t(ModuleStateKind::Loaded));
    EXPECT_EQ(rig.m7.states[0].crc, image.checksum());
    EXPECT_EQ(rig.m7.states[0].rx_size, SplitImportContainer::code_size);
    EXPECT_EQ(rig.m7.states[1].state, uint8_t(ModuleStateKind::Started));
    const std::vector<std::string> events { "list", "close", "resolve", "state:1", "state:2", "start" };
    EXPECT_EQ(rig.m7.events, events);

    // the remote import's name, for the Stats rows; the local one is answered on the M4
    EXPECT_EQ(rig.names.size(), 1U);
    EXPECT_EQ(rig.names[remote_id()], "remote.api");
}

static void expect_failed(const LoaderRig &rig, const std::string &reason)
{
    EXPECT_EQ(rig.host.state(), ModuleHost::Step::Failed);
    EXPECT_EQ(std::string(rig.host.failure()), reason);
    EXPECT_FALSE(started(rig));
    ASSERT_FALSE(rig.m7.states.empty());
    EXPECT_EQ(rig.m7.states.back().state, uint8_t(ModuleStateKind::Failed));
    EXPECT_EQ(std::string(rig.m7.states.back().text), reason);
}

TEST(ModuleM4ModuleHost, TruncatedFileFailsParse)
{
    SplitImportContainer container;
    std::vector<uint8_t> truncated(container.bytes.begin(), container.bytes.end() - 1);
    Image image;
    EXPECT_FALSE(image.parse(truncated.data(), truncated.size(), 131072, parse_compatibility()));
    LoaderRig rig(truncated);
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: " + image.parse_error());
}

TEST(ModuleM4ModuleHost, FlippedByteFailsParse)
{
    SplitImportContainer container;
    std::vector<uint8_t> flipped = container.bytes;
    flipped[SplitImportContainer::header_size + 100] ^= 0x01;
    LoaderRig rig(flipped);   // FsClose reports the CRC of what it served
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: image checksum mismatch");
    EXPECT_EQ(rig.steps.back(), ModuleHost::Step::Failed);
    EXPECT_EQ(rig.steps[rig.steps.size() - 2], ModuleHost::Step::Parse);
}

TEST(ModuleM4ModuleHost, ServedCrcDisagreesFailsClosed)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.crc_offset = 1;
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: checksum mismatch");
    EXPECT_EQ(rig.steps[rig.steps.size() - 2], ModuleHost::Step::Checksum);
}

TEST(ModuleM4ModuleHost, UnknownImportFails)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.ids[REMOTE_HASH] = UINT32_MAX;
    rig.run();
    expect_failed(rig, std::string("missing API remote.api in ") + FILE_NAME + " [1122334455667788]");
    EXPECT_TRUE(rig.names.empty());
}

TEST(ModuleM4ModuleHost, LocalCatalogueDriftFails)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.ids[AP_Module::M4_LOCAL_IMPORTS[0].hash] = remote_id();
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: local binding catalogue mismatch");
}

TEST(ModuleM4ModuleHost, RemoteHashResolvedToALocalIdFails)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.ids[REMOTE_HASH] = AP_Module::M4_LOCAL_IMPORTS[0].id;
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: local binding catalogue mismatch");
}

TEST(ModuleM4ModuleHost, TooLargeForTheAreaFails)
{
    SplitImportContainer container;
    const uint32_t area_size = 12288;   // the code window takes 8 KB: the file cannot be staged below it
    LoaderRig rig(container.bytes, area_size);
    rig.m7.config.code_size = 8192;
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: insufficient M4 RAM to load");
    EXPECT_EQ(memcmp(rig.platform.area, container.bytes.data(), container.bytes.size()), 0);
    for (uint32_t i = uint32_t(container.bytes.size()); i < area_size; i++) {
        ASSERT_EQ(rig.platform.area[i], 0xCC) << "written above staging at " << i;
    }
}

TEST(ModuleM4ModuleHost, ListedSizeAboveTheAreaFails)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.listed_size = AP_MODULE_M4_MODULE_AREA_SIZE + 1U;
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: insufficient M4 RAM to load");
    EXPECT_EQ(rig.m7.offset, 0U);
}

TEST(ModuleM4ModuleHost, HeapAboveTheBitmapsFails)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.config.heap_size = int32_t(MODULE_HEAP_MAX + 16U);
    rig.run();
    expect_failed(rig, std::string(FILE_NAME) + " load failed: insufficient M4 RAM to load");
}

TEST(ModuleM4ModuleHost, NoModuleIsIdle)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.m7.list_count = 0;
    rig.run();
    EXPECT_EQ(rig.host.state(), ModuleHost::Step::Idle);
    EXPECT_TRUE(rig.m7.states.empty());
    EXPECT_EQ(rig.m7.offset, 0U);
}

TEST(ModuleM4ModuleHost, RefusedStartFails)
{
    SplitImportContainer container;
    LoaderRig rig(container.bytes);
    rig.platform.start = [](void *, ModuleContext &, const LoadLayout &, uint32_t, uint32_t) {};
    rig.host.init(rig.platform);
    rig.run();
    EXPECT_EQ(rig.host.state(), ModuleHost::Step::Failed);
    EXPECT_STREQ(rig.host.failure(), "module heap/thread creation failed");
}

AP_GTEST_MAIN()

#else
TEST(ModuleM4ModuleHost, NeedsRust) {}
AP_GTEST_MAIN()
#endif  // AP_MODULE_ENABLED
