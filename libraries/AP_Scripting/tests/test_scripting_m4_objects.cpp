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

// the M4 calling a userdata's __gc twice (by hand, or a finalizer run again): one release
// reaches the M7, and it is no protocol error. Without the generation reset in push() the same
// handle goes out twice and the second makes the m4:release Malformed
TEST(ScriptingM4Objects, SecondFinalizerSendsNoRelease)
{
    HandleTable handles;
    handles.reset(4);
    ObjectPool pool;
    destroyed = 0;
    const int16_t index = handles.insert_owned(5, pool.allocate(count_destroy));
    ASSERT_GE(index, 0);
    uint32_t userdata_generation = handles.generation(uint16_t(index));
    ReleaseQueue queue;
    EXPECT_TRUE(queue.push(5, uint16_t(index), userdata_generation));
    EXPECT_EQ(userdata_generation, 0U);
    EXPECT_FALSE(queue.push(5, uint16_t(index), userdata_generation));
    EXPECT_EQ(queue.count(), 1);
    int sends = 0;
    ReplyStatus status = ReplyStatus::Error;
    queue.flush([&](const ReleaseQueue::Entry *entries, uint8_t count) {
        RpcValue values[MAX_VALUES];
        for (uint8_t i = 0; i < count; i++) {
            values[i] = handle_value(entries[i].type_id, int16_t(entries[i].index), entries[i].generation);
        }
        status = release_objects(values, count, handles, pool);
        sends++;
        return true;
    });
    EXPECT_EQ(sends, 1);
    EXPECT_EQ(status, ReplyStatus::Ok);
    EXPECT_EQ(destroyed, 1);
    EXPECT_EQ(queue.count(), 0);
    // the zeroed userdata, used again, names no object
    EXPECT_EQ(handles.lookup(5, uint16_t(index), userdata_generation), nullptr);
}

// a send that fails (the RPC call timed out) keeps its batch and the rest for the next flush;
// without that, the M7 keeps those objects until the session ends
TEST(ScriptingM4Objects, ReleaseQueueKeepsWhatItCouldNotSend)
{
    ReleaseQueue queue;
    const uint8_t total = MAX_VALUES + 4;
    for (uint8_t i = 0; i < total; i++) {
        uint32_t generation = 0x700 + i;
        ASSERT_TRUE(queue.push(3, i, generation));
    }
    uint8_t calls = 0;
    queue.flush([&](const ReleaseQueue::Entry *entries, uint8_t count) {
        calls++;
        if (calls == 1) {
            EXPECT_EQ(count, MAX_VALUES);
            EXPECT_EQ(entries[0].index, 0);
            return true;
        }
        return false;
    });
    EXPECT_EQ(calls, 2);
    ASSERT_EQ(queue.count(), 4);
    uint16_t first = 0;
    uint32_t first_generation = 0;
    queue.flush([&](const ReleaseQueue::Entry *entries, uint8_t count) {
        EXPECT_EQ(count, 4);
        first = entries[0].index;
        first_generation = entries[0].generation;
        return true;
    });
    EXPECT_EQ(first, MAX_VALUES);
    EXPECT_EQ(first_generation, 0x700U + MAX_VALUES);
    EXPECT_EQ(queue.count(), 0);
}

// a full queue drops and counts; without the bound the 65th entry is written past the queue
TEST(ScriptingM4Objects, FullReleaseQueueCountsDrops)
{
    ReleaseQueue queue;
    const uint8_t slots = ObjectPool::SLOTS;
    for (uint8_t i = 0; i < slots; i++) {
        uint32_t generation = 0x100 + i;
        ASSERT_TRUE(queue.push(2, i, generation));
    }
    uint32_t generation = 0x200;
    EXPECT_FALSE(queue.push(2, 100, generation));
    EXPECT_EQ(queue.count(), slots);
    EXPECT_EQ(queue.dropped(), 1U);
}

TEST(ScriptingM4Objects, ObjectPoolContainsItsSlotsOnly)
{
    ObjectPool pool;
    uint8_t *slot = static_cast<uint8_t *>(pool.allocate(count_destroy));
    ASSERT_NE(slot, nullptr);
    int outside = 0;
    EXPECT_TRUE(pool.contains(slot));
    EXPECT_TRUE(pool.contains(slot + 1));                       // inside the object
    EXPECT_TRUE(pool.contains(slot + ObjectPool::SLOT_SIZE));   // a free slot
    EXPECT_FALSE(pool.contains(&outside));
    EXPECT_FALSE(pool.contains(nullptr));
}

TEST(ScriptingM4Objects, ReleaseOfAnEarlierSessionsHandleDestroysNothing)
{
    HandleTable handles;
    ObjectPool pool;
    handles.reset(1);
    destroyed = 0;
    void *old_object = pool.allocate(count_destroy);
    const int16_t old_index = handles.insert_owned(3, old_object);
    ASSERT_GE(old_index, 0);
    const RpcValue old_handle = handle_value(3, old_index, handles.generation(uint16_t(old_index)));
    // the session ends: pool and table are emptied, the next session starts
    pool.clear();
    handles.reset(2);
    EXPECT_EQ(destroyed, 1);
    void *new_object = pool.allocate(count_destroy);
    ASSERT_EQ(new_object, old_object);   // the same slot, now another session's object
    ASSERT_GE(handles.insert_owned(3, new_object), 0);
    // the old handle names the old entry index but not its generation: nothing is destroyed
    EXPECT_EQ(release_objects(&old_handle, 1, handles, pool), ReplyStatus::Malformed);
    EXPECT_EQ(destroyed, 1);
    EXPECT_EQ(pool.used(), 1);
}

TEST(ScriptingM4Objects, RecyclerHandsOutEarlierSessionsSources)
{
    static int sources[6];
    int made = 0;
    auto make = [&]() -> int * { return made < 6 ? &sources[made++] : nullptr; };
    SessionRecycler<int, 4> recycler;
    EXPECT_EQ(recycler.take(make), &sources[0]);
    EXPECT_EQ(recycler.take(make), &sources[1]);
    EXPECT_EQ(made, 2);
    recycler.new_session();
    // the next session gets the same two before anything new: without the recycler the HAL
    // would hand out two more of its slots per M4 restart
    EXPECT_EQ(recycler.take(make), &sources[0]);
    EXPECT_EQ(recycler.take(make), &sources[1]);
    EXPECT_EQ(made, 2);
    EXPECT_EQ(recycler.take(make), &sources[2]);
    EXPECT_EQ(made, 3);
    EXPECT_EQ(recycler.count(), 3);
    // past its capacity a source is still handed out, only not kept for later sessions
    EXPECT_EQ(recycler.take(make), &sources[3]);
    EXPECT_EQ(recycler.take(make), &sources[4]);
    EXPECT_EQ(recycler.count(), 4);
    recycler.new_session();
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(recycler.take(make), &sources[i]) << i;
    }
    EXPECT_EQ(recycler.take(make), &sources[5]);
    EXPECT_EQ(recycler.take(make), nullptr);   // the HAL ran out: nil, as in-process
}

// the scripting CAN buffers (CAN:get_device): the sensor's buffer list is never shortened and the
// CAN thread walks it recursively, so an M4 session gets at most 8 and earlier sessions' buffers
// are handed out again
TEST(ScriptingM4Objects, CappedRecyclerRefusesPastItsCapacity)
{
    static int buffers[16];
    int made = 0;
    auto make = [&made]() { return &buffers[made++]; };
    SessionRecycler<int, 8> recycler;
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(recycler.take_at_most(make), &buffers[i]) << i;
    }
    // without the cap a 9th buffer is made, and a script's loop makes them until the heap is gone
    EXPECT_EQ(recycler.take_at_most(make), nullptr);
    EXPECT_EQ(made, 8);                          // refused before make(): the M4's retry repeats nothing
    recycler.new_session();
    // without the recycling every M4 restart adds 8 more buffers to the sensor's list
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(recycler.take_at_most(make), &buffers[i]) << i;
    }
    EXPECT_EQ(recycler.take_at_most(make), nullptr);
    EXPECT_EQ(made, 8);
    EXPECT_EQ(recycler.count(), 8);
}

TEST(ScriptingM4Objects, CappedRecyclerKeepsNothingMakeRefused)
{
    int made = 0;
    auto refuse = [&made]() { made++; return static_cast<int *>(nullptr); };
    SessionRecycler<int, 2> recycler;
    EXPECT_EQ(recycler.take_at_most(refuse), nullptr);   // out of memory: nil, as in-process
    EXPECT_EQ(recycler.count(), 0);
    static int buffer;
    EXPECT_EQ(recycler.take_at_most([]() { return &buffer; }), &buffer);
    EXPECT_EQ(made, 1);
}

AP_GTEST_MAIN()
