#define AP_PARAM_VEHICLE_NAME testvehicle

// the values compared are set exactly
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <AP_gtest.h>
#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED
#include <pthread.h>
#include <string.h>
#include <vector>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/HostApi.h>
#include <AP_ModuleM4/AP_ModuleM4_Server.h>
#include <AP_Vehicle/AP_Vehicle.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

using namespace ModuleM4;
using AP_Module::BindingId;

class Parameters
{
public:
    enum { k_param_watch, k_param_other, k_param_f0, k_param_f1, k_param_f2, k_param_f3 };
    AP_Float watch;
    AP_Float other;
    AP_Float f0, f1, f2, f3;
};

class TestVehicle : public AP_Vehicle
{
public:
    TestVehicle()
    {
        unused_log_bitmask.set(-1);
    }
    void load_parameters() override {}
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks, uint8_t &count, uint32_t &log_bit) override
    {
        tasks = nullptr;
        count = 0;
        log_bit = 0;
    }
    bool set_mode(uint8_t, ModeReason) override
    {
        return true;
    }
    uint8_t get_mode() const override
    {
        return 0;
    }
    static const AP_Param::Info var_info[];
    Parameters g;
    AP_Param param_loader{var_info};

protected:
    void init_ardupilot() override {}
    const AP_UInt32 &get_log_bitmask() override
    {
        return unused_log_bitmask;
    }
    const LogStructure *get_log_structures() const override
    {
        return nullptr;
    }
    uint8_t get_num_log_structures() const override
    {
        return 0;
    }

private:
    AP_UInt32 unused_log_bitmask;
};
static TestVehicle testvehicle;

const AP_Param::Info TestVehicle::var_info[] = {
    GSCALAR(watch, "WATCH", 3.5f),
    GSCALAR(other, "OTHER", 1.0f),
    GSCALAR(f0, "F0", 0), GSCALAR(f1, "F1", 0), GSCALAR(f2, "F2", 0), GSCALAR(f3, "F3", 0),
    AP_VAREND
};

static AP_ModuleM4_Server server;   // large: static storage
static const uint64_t T1 = 0x20001000ULL, T2 = 0x20001010ULL, T3 = 0x20001020ULL;

static int32_t serve(uint16_t module, BindingId id, const uint8_t *input, uint16_t in_size, uint16_t out_size)
{
    static uint8_t frame[MAX_FRAME], reply[MAX_FRAME];
    const uint16_t len = pack_binding_call(frame, MAX_FRAME, 42, module, 0, uint32_t(id), input, in_size, out_size);
    const uint16_t reply_len = server.serve_binding(frame, len, reply, sizeof(reply), 1000);
    BindingReplyHeader h {};
    const uint8_t *body = nullptr;
    EXPECT_TRUE(parse_binding_reply(reply, reply_len, h, body));
    return h.status;
}

static int32_t init(uint16_t module, uint64_t token, const char *name)
{
    uint8_t input[8 + 17] {};
    memcpy(input, &token, sizeof(token));
    strncpy(reinterpret_cast<char *>(input + 8), name, 16);
    return serve(module, BindingId::PARAMETER_INIT, input, sizeof(input), 1);
}

static int32_t set_by_handle(uint16_t module, uint64_t token, float value)
{
    uint8_t input[12] {};
    memcpy(input, &token, sizeof(token));
    memcpy(input + 8, &value, sizeof(value));
    return serve(module, BindingId::PARAMETER_SET, input, sizeof(input), 1);
}

static int32_t set_by_name(uint16_t module, const char *name, float value)
{
    uint8_t input[17 + 4] {};
    strncpy(reinterpret_cast<char *>(input), name, 16);
    memcpy(input + 17, &value, sizeof(value));
    return serve(module, BindingId::PARAM_SET, input, sizeof(input), 1);
}

// every entry the server packs until it has none left
static std::vector<ParameterValue> drain()
{
    std::vector<ParameterValue> entries;
    uint8_t frame[MAX_FRAME];
    while (true) {
        const uint16_t len = server.pack_mirror_changes(frame, sizeof(frame));
        if (len == 0) {
            return entries;
        }
        ParameterChangedHeader h;
        EXPECT_TRUE(parse_parameter_changed(frame, len, h));
        EXPECT_EQ(h.header.seq, 7U);
        for (uint16_t i = 0; i < h.count; i++) {
            entries.push_back(parameter_value_at(frame, i));
        }
    }
}

static const ParameterValue *find(const std::vector<ParameterValue> &entries, uint16_t module, uint64_t token)
{
    for (const auto &e : entries) {
        if (e.module == module && e.token == token) {
            return &e;
        }
    }
    return nullptr;
}

class ParamMirrorFeed : public ::testing::Test
{
protected:
    void SetUp() override
    {
        server.reset(7, 2);
        server.run_cleanups_for_test(false);
        testvehicle.g.watch.set(3.5f);
        testvehicle.g.other.set(1.0f);
    }
};

TEST_F(ParamMirrorFeed, InitBindsAndSendsItsEntry)
{
    ASSERT_EQ(init(0, T1, "WATCH"), STATUS_OK);
    EXPECT_TRUE(server.last_call_wrote_parameters());
    EXPECT_EQ(server.bound_param_for_test(0, T1), &testvehicle.g.watch);
    server.mark_all_live();
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].token, T1);
    EXPECT_EQ(entries[0].value, 3.5f);
    EXPECT_EQ(entries[0].flags, PARAMETER_VALUE_BOUND);
    EXPECT_TRUE(drain().empty());
}

TEST_F(ParamMirrorFeed, MarkChangedSetsTheBitsOfEveryBoundSlot)
{
    init(0, T1, "WATCH");
    init(0, T2, "WATCH");
    init(0, T3, "OTHER");
    server.mark_all_live();
    drain();
    testvehicle.g.watch.set(4.0f);
    server.mark_changed(&testvehicle.g.watch);
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 2U);
    EXPECT_NE(find(entries, 0, T1), nullptr);
    EXPECT_NE(find(entries, 0, T2), nullptr);
    EXPECT_EQ(find(entries, 0, T1)->value, 4.0f);
}

TEST_F(ParamMirrorFeed, UnchangedValuesAreSkipped)
{
    init(0, T1, "WATCH");
    server.mark_all_live();
    drain();
    server.mark_changed(&testvehicle.g.watch);
    EXPECT_TRUE(drain().empty());
    server.mark_changed(nullptr);
    EXPECT_FALSE(server.mirror_dirty());
}

static void *mark_many(void *arg)
{
    auto *param = static_cast<const AP_Param *>(arg);
    for (uint32_t i = 0; i < 20000; i++) {
        server.mark_changed(param);
    }
    return nullptr;
}

TEST_F(ParamMirrorFeed, ConcurrentProducersLoseNothing)
{
    AP_Float *params[4] = { &testvehicle.g.f0, &testvehicle.g.f1, &testvehicle.g.f2, &testvehicle.g.f3 };
    const char *names[4] = { "F0", "F1", "F2", "F3" };
    for (uint16_t i = 0; i < 4; i++) {
        init(0, T1 + 16U * i, names[i]);
    }
    server.mark_all_live();
    drain();
    for (uint16_t i = 0; i < 4; i++) {
        params[i]->set(10.0f + i);
    }
    pthread_t threads[4];
    for (uint16_t i = 0; i < 4; i++) {
        ASSERT_EQ(pthread_create(&threads[i], nullptr, mark_many, params[i]), 0);
    }
    // drains race the producers: each changed slot is sent exactly once
    std::vector<ParameterValue> entries;
    for (uint16_t round = 0; round < 50; round++) {
        const auto some = drain();
        entries.insert(entries.end(), some.begin(), some.end());
    }
    for (auto &thread : threads) {
        pthread_join(thread, nullptr);
    }
    const auto rest = drain();
    entries.insert(entries.end(), rest.begin(), rest.end());
    ASSERT_EQ(entries.size(), 4U);
    for (uint16_t i = 0; i < 4; i++) {
        ASSERT_NE(find(entries, 0, T1 + 16U * i), nullptr) << i;
        EXPECT_EQ(find(entries, 0, T1 + 16U * i)->value, 10.0f + i);
    }
}

TEST_F(ParamMirrorFeed, HandleSetSettlesOtherHandlesOfTheSameParameter)
{
    init(0, T1, "WATCH");
    init(0, T2, "WATCH");
    server.mark_all_live();
    drain();
    ASSERT_EQ(set_by_handle(0, T1, 6.0f), STATUS_OK);
    EXPECT_TRUE(server.last_call_wrote_parameters());
    server.mark_all_live();   // what AP_ModuleM4 does before the reply
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 2U);
    EXPECT_EQ(find(entries, 0, T2)->value, 6.0f);
}

TEST_F(ParamMirrorFeed, ByNameSetIsSettled)
{
    init(0, T1, "WATCH");
    server.mark_all_live();
    drain();
    ASSERT_EQ(set_by_name(0, "WATCH", 8.0f), STATUS_OK);
    EXPECT_TRUE(server.last_call_wrote_parameters());
    server.mark_all_live();
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].value, 8.0f);
}

TEST_F(ParamMirrorFeed, HandleWriterAlwaysSendsItsOwnEntry)
{
    init(0, T1, "WATCH");
    server.mark_all_live();
    drain();
    ASSERT_EQ(set_by_handle(0, T1, NAN), STATUS_INVALID);   // rejected: value unchanged
    server.mark_all_live();
    auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].value, 3.5f);
    ASSERT_EQ(set_by_handle(0, T1, 3.5f), STATUS_OK);   // the same value
    server.mark_all_live();
    entries = drain();
    ASSERT_EQ(entries.size(), 1U);
}

TEST_F(ParamMirrorFeed, InitOfAReusedTokenSendsItsEntry)
{
    testvehicle.g.other.set(3.5f);   // the same value as WATCH
    init(0, T1, "WATCH");
    server.mark_all_live();
    drain();
    init(0, T1, "OTHER");
    EXPECT_EQ(server.bound_param_for_test(0, T1), &testvehicle.g.other);
    server.mark_all_live();
    ASSERT_EQ(drain().size(), 1U);
}

TEST_F(ParamMirrorFeed, FailedInitSendsNoEntry)
{
    for (uint16_t i = 0; i < AP_ModuleM4_Server::PARAMETER_HANDLES_MAX; i++) {
        ASSERT_EQ(init(0, T1 + 16U * i, "WATCH"), STATUS_OK) << i;
    }
    EXPECT_EQ(init(0, T1 + 16U * AP_ModuleM4_Server::PARAMETER_HANDLES_MAX, "WATCH"), STATUS_NO_MEMORY);
    server.mark_all_live();
    const auto entries = drain();
    EXPECT_EQ(entries.size(), size_t(AP_ModuleM4_Server::PARAMETER_HANDLES_MAX));
    EXPECT_EQ(find(entries, 0, T1 + 16U * AP_ModuleM4_Server::PARAMETER_HANDLES_MAX), nullptr);
}

TEST_F(ParamMirrorFeed, FailedSendPutsTheBitsBackAndOwesAResync)
{
    init(0, T1, "WATCH");
    server.mark_all_live();
    drain();
    testvehicle.g.watch.set(5.0f);
    server.mark_changed(&testvehicle.g.watch);
    uint8_t frame[MAX_FRAME];
    const uint16_t len = server.pack_mirror_changes(frame, sizeof(frame));
    ASSERT_NE(len, 0U);
    server.mirror_send_failed(frame, len);
    EXPECT_TRUE(server.take_resync(0));
    EXPECT_FALSE(server.take_resync(0));
    EXPECT_FALSE(server.take_resync(1));
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].value, 5.0f);
}

TEST_F(ParamMirrorFeed, EraseMoveKeepsThePendingChange)
{
    // T2 of module 1 shares T1's home slot: it sits one slot later until module 0 ends
    const uint16_t home = AP_ModuleM4_Server::handle_home_for_test(0, T1);
    uint64_t t2 = 0x30000000ULL;
    while (AP_ModuleM4_Server::handle_home_for_test(1, t2) != home) {
        t2 += 16;
    }
    init(0, T1, "OTHER");
    init(1, t2, "WATCH");
    server.mark_all_live();
    drain();
    testvehicle.g.watch.set(7.0f);
    server.mark_changed(&testvehicle.g.watch);
    server.end_module(0);   // erase_handle() moves module 1's entry into the home slot
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].module, 1U);
    EXPECT_EQ(entries[0].token, t2);
    EXPECT_EQ(entries[0].value, 7.0f);
}

TEST_F(ParamMirrorFeed, BackstopFindsARawSet)
{
    init(0, T1, "WATCH");
    server.mark_all_live();
    drain();
    testvehicle.g.watch.set(9.0f);   // no save, no notify: not observed
    EXPECT_TRUE(drain().empty());
    server.mark_all_live();
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].value, 9.0f);
}

TEST_F(ParamMirrorFeed, SnapshotSendsEveryLiveSlotOfTheModule)
{
    init(0, T1, "WATCH");
    init(1, T2, "WATCH");
    server.mark_all_live();
    drain();
    server.snapshot_module(0);
    const auto entries = drain();
    ASSERT_EQ(entries.size(), 1U);
    EXPECT_EQ(entries[0].module, 0U);
}

TEST_F(ParamMirrorFeed, EndAndResetForgetSlots)
{
    init(0, T1, "WATCH");
    server.end_module(0);
    EXPECT_EQ(server.bound_param_for_test(0, T1), nullptr);
    server.mark_changed(&testvehicle.g.watch);
    EXPECT_TRUE(drain().empty());
    init(1, T2, "WATCH");
    server.reset(7, 2);
    EXPECT_FALSE(server.mirror_dirty());
    EXPECT_FALSE(server.take_resync(1));
}

#endif  // AP_MODULE_ENABLED

AP_GTEST_MAIN()
