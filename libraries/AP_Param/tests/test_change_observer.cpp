#define AP_PARAM_VEHICLE_NAME testvehicle

#include <AP_gtest.h>
#include <AP_HAL_Empty/Scheduler.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <string.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#if AP_PARAM_CHANGE_OBSERVER_ENABLED

class Parameters
{
public:
    enum { k_param_value, k_param_float_value };
    AP_Int32 value;
    AP_Float float_value;
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
    GSCALAR(value, "VALUE", 0),
    GSCALAR(float_value, "FLOAT", 0),
    AP_VAREND
};

// notify() and the IO thread's save_sync() send to the GCS; this test has no links
class TestGCS : public GCS
{
public:
    uint32_t custom_mode() const override
    {
        return 0;
    }
    MAV_TYPE frame_type() const override
    {
        return MAV_TYPE_GENERIC;
    }
    GCS_MAVLINK *chan(uint8_t) override
    {
        return nullptr;
    }
    const GCS_MAVLINK *chan(uint8_t) const override
    {
        return nullptr;
    }
    GCS_MAVLINK *new_gcs_mavlink_backend(AP_HAL::UARTDriver &) override
    {
        return nullptr;
    }
};
static TestGCS test_gcs;

// keeps the IO callback AP_Param registers once, so a test runs the queued saves itself
class IoScheduler : public Empty::Scheduler
{
public:
    void register_io_process(AP_HAL::MemberProc proc) override
    {
        io = proc;
    }
    bool is_system_initialized() override
    {
        return false;
    }
    bool in_main_thread() const override
    {
        return true;
    }
    AP_HAL::MemberProc io;
};
static IoScheduler scheduler;

class MemoryStorage : public AP_HAL::Storage
{
public:
    void init() override {}
    void read_block(void *dst, uint16_t src, size_t size) override
    {
        ASSERT_LE(size_t(src) + size, sizeof(bytes));
        memcpy(dst, &bytes[src], size);
    }
    void write_block(uint16_t dst, const void *src, size_t size) override
    {
        ASSERT_LE(size_t(dst) + size, sizeof(bytes));
        memcpy(&bytes[dst], src, size);
    }

private:
    uint8_t bytes[HAL_STORAGE_SIZE] {};
};

static uint32_t observed;
static const AP_Param *last_observed;
static void count_observation(const AP_Param *param)
{
    observed++;
    last_observed = param;
}
static uint32_t other_observed;
static void other_observer(const AP_Param *)
{
    other_observed++;
}

class ChangeObserver : public ::testing::Test
{
protected:
    void SetUp() override
    {
        AP_HAL::get_HAL_mutable().scheduler = &scheduler;
        AP_HAL::get_HAL_mutable().storage = &storage;
        AP_Param::erase_all();
        AP_Param::load_all();
        ASSERT_TRUE(bool(scheduler.io));
        scheduler.io();   // an empty queue for every test
        ASSERT_TRUE(AP_Param::set_change_observer(nullptr));
        observed = 0;
        other_observed = 0;
        last_observed = nullptr;
    }
    void TearDown() override
    {
        scheduler.io();
        IGNORE_RETURN(AP_Param::set_change_observer(nullptr));
        AP_HAL::get_HAL_mutable().storage = old_storage;
        AP_HAL::get_HAL_mutable().scheduler = old_scheduler;
    }

    AP_HAL::Scheduler *old_scheduler = hal.scheduler;
    AP_HAL::Storage *old_storage = hal.storage;
    MemoryStorage storage;
};

TEST_F(ChangeObserver, SaveIsObservedBeforeTheDedupe)
{
    ASSERT_TRUE(AP_Param::set_change_observer(count_observation));
    testvehicle.g.value.set(5);
    testvehicle.g.value.save();
    testvehicle.g.value.set(6);
    testvehicle.g.value.save();   // deduped: already at the head of the queue
    EXPECT_EQ(observed, 2U);
    EXPECT_EQ(last_observed, &testvehicle.g.value);
    // one entry was queued: its write on the IO thread is the third observation
    scheduler.io();
    EXPECT_EQ(observed, 3U);
}

TEST_F(ChangeObserver, SaveSyncIsObserved)
{
    ASSERT_TRUE(AP_Param::set_change_observer(count_observation));
    testvehicle.g.float_value.set(2.5f);
    testvehicle.g.float_value.save_sync(false, false);   // as the MAVFTP @PARAM upload
    EXPECT_EQ(observed, 1U);
    EXPECT_EQ(last_observed, &testvehicle.g.float_value);
}

TEST_F(ChangeObserver, NotifyIsObserved)
{
    ASSERT_TRUE(AP_Param::set_change_observer(count_observation));
    testvehicle.g.value.set_and_notify(9);
    EXPECT_EQ(observed, 1U);
    EXPECT_EQ(last_observed, &testvehicle.g.value);
}

TEST_F(ChangeObserver, SetIsNotObserved)
{
    ASSERT_TRUE(AP_Param::set_change_observer(count_observation));
    testvehicle.g.value.set(11);
    testvehicle.g.float_value.set_float(1.5f, AP_PARAM_FLOAT);
    EXPECT_EQ(observed, 0U);
}

TEST_F(ChangeObserver, NoObserverIsSafe)
{
    testvehicle.g.value.set_and_save(12);
    testvehicle.g.value.notify();
    scheduler.io();
    EXPECT_EQ(observed, 0U);
}

TEST_F(ChangeObserver, SecondObserverIsRefused)
{
    ASSERT_TRUE(AP_Param::set_change_observer(count_observation));
    EXPECT_FALSE(AP_Param::set_change_observer(other_observer));
    testvehicle.g.value.set_and_save(13);
    EXPECT_EQ(observed, 1U);
    EXPECT_EQ(other_observed, 0U);
    EXPECT_TRUE(AP_Param::set_change_observer(nullptr));
    EXPECT_TRUE(AP_Param::set_change_observer(other_observer));
}

#endif  // AP_PARAM_CHANGE_OBSERVER_ENABLED

AP_GTEST_MAIN()
