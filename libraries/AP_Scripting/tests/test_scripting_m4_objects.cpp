#include <AP_gtest.h>
#include <string.h>
#include <AP_Scripting/AP_ScriptingM4_Objects.h>

using namespace ScriptingM4;

static int destroyed;
static void count_destroy(void *object)
{
    destroyed++;
    memset(object, 0xA5, ObjectPool::SLOT_SIZE);   // a destructor may scribble over its own slot
}

static RpcValue handle_value(uint8_t type_id, int16_t index, uint32_t generation)
{
    RpcValue value {};
    value.tag = ValueTag::Handle;
    value.type_id = type_id;
    value.handle_index = uint16_t(index);
    value.handle_generation = generation;
    return value;
}

TEST(ScriptingM4Objects, PoolHandsOutZeroedAlignedSlots)
{
    ObjectPool pool;
    void *slots[ObjectPool::SLOTS];
    static const uint8_t zeros[ObjectPool::SLOT_SIZE] = {};
    for (uint8_t i = 0; i < ObjectPool::SLOTS; i++) {
        slots[i] = pool.allocate(count_destroy);
        ASSERT_NE(slots[i], nullptr) << int(i);
        EXPECT_EQ(reinterpret_cast<uintptr_t>(slots[i]) % ObjectPool::SLOT_ALIGN, 0U);
        EXPECT_EQ(memcmp(slots[i], zeros, sizeof(zeros)), 0);
        memset(slots[i], 0x5A, ObjectPool::SLOT_SIZE);
    }
    EXPECT_EQ(pool.used(), uint8_t(ObjectPool::SLOTS));
    EXPECT_EQ(pool.allocate(count_destroy), nullptr);   // full: without the bound a 65th slot
    EXPECT_EQ(pool.allocate(nullptr), nullptr);         // an object nothing could destroy
    destroyed = 0;
    EXPECT_TRUE(pool.destroy(slots[3]));
    EXPECT_EQ(destroyed, 1);
    void *again = pool.allocate(count_destroy);
    EXPECT_EQ(again, slots[3]);                          // the freed slot, zeroed again
    EXPECT_EQ(memcmp(again, zeros, sizeof(zeros)), 0);
    pool.clear();
    EXPECT_EQ(destroyed, 1 + ObjectPool::SLOTS);
    EXPECT_EQ(pool.used(), 0);
}

TEST(ScriptingM4Objects, DestroyRefusesWhatThePoolDidNotHandOut)
{
    ObjectPool pool;
    destroyed = 0;
    uint8_t *slot = static_cast<uint8_t *>(pool.allocate(count_destroy));
    ASSERT_NE(slot, nullptr);
    int outside = 0;
    // without the address checks each of these would run a destructor on memory it does not own
    EXPECT_FALSE(pool.destroy(&outside));
    EXPECT_FALSE(pool.destroy(slot + 1));               // inside a slot, not its start
    EXPECT_FALSE(pool.destroy(slot + ObjectPool::SLOT_SIZE));   // a free slot
    EXPECT_FALSE(pool.destroy(nullptr));
    EXPECT_TRUE(pool.destroy(slot));
    EXPECT_FALSE(pool.destroy(slot));                   // twice
    EXPECT_EQ(destroyed, 1);
}

TEST(ScriptingM4Objects, ReleaseDestroysOnlyLiveOwnedObjects)
{
    HandleTable handles;
    handles.reset(9);
    ObjectPool pool;
    destroyed = 0;
    void *a = pool.allocate(count_destroy);
    void *b = pool.allocate(count_destroy);
    int kept_object = 0;
    const int16_t ia = handles.insert_owned(5, a);
    const int16_t ib = handles.insert_owned(5, b);
    const int16_t ik = handles.insert(6, &kept_object);
    ASSERT_GE(ia, 0);
    ASSERT_GE(ib, 0);
    ASSERT_GE(ik, 0);
    RpcValue values[3];
    values[0] = handle_value(5, ia, handles.generation(ia));
    EXPECT_EQ(release_objects(values, 1, handles, pool), ReplyStatus::Ok);
    EXPECT_EQ(destroyed, 1);
    // the same handle again (a finalizer that ran twice, or a forged repeat): nothing more
    EXPECT_EQ(release_objects(values, 1, handles, pool), ReplyStatus::Malformed);
    EXPECT_EQ(destroyed, 1);
    // a kept handle, a live owned index under another type, and a value that is no handle: all
    // refused; without the owned check the kept entry would be retired
    values[0] = handle_value(6, ik, handles.generation(ik));
    values[1] = handle_value(6, ib, handles.generation(ib));
    values[2] = RpcValue {};
    values[2].tag = ValueTag::Integer;
    values[2].integer = ib;
    EXPECT_EQ(release_objects(values, 3, handles, pool), ReplyStatus::Malformed);
    EXPECT_EQ(destroyed, 1);
    EXPECT_EQ(handles.lookup(6, ik, handles.generation(ik)), &kept_object);
    EXPECT_EQ(handles.lookup(5, ib, handles.generation(ib)), b);
    EXPECT_EQ(release_objects(values, 0, handles, pool), ReplyStatus::Malformed);
}

TEST(ScriptingM4Objects, ReleaseManyInOneCall)
{
    HandleTable handles;
    handles.reset(2);
    ObjectPool pool;
    destroyed = 0;
    RpcValue values[MAX_VALUES];
    for (uint8_t i = 0; i < MAX_VALUES; i++) {
        const int16_t index = handles.insert_owned(3, pool.allocate(count_destroy));
        ASSERT_GE(index, 0);
        values[i] = handle_value(3, index, handles.generation(index));
    }
    EXPECT_EQ(release_objects(values, MAX_VALUES, handles, pool), ReplyStatus::Ok);
    EXPECT_EQ(destroyed, MAX_VALUES);
    EXPECT_EQ(pool.used(), 0);
    EXPECT_EQ(handles.count(), 0);
}

AP_GTEST_MAIN()
