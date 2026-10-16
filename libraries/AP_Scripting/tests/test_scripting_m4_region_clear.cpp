#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_NetRegionClear.h>
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
#include <AP_Networking/AP_Networking_M4_Layout.h>
#endif

using namespace ScriptingM4;

namespace {
// the PeerHeader::View fields around the fault count, each a different value: the helper must read
// faultCount, not a neighbour
struct Header {
    uint32_t restarts;
    uint32_t session;
    uint32_t faultCount;
    uint32_t faultPc;
};
Header header(uint32_t fault_count)
{
    return Header { fault_count + 100U, fault_count + 200U, fault_count, fault_count + 300U };
}
}

// the running session faults: it waits in its fault handler, the grant clears the region
TEST(ScriptingM4NetRegionClear, OwnFaultClears)
{
    NetRegionClear clear;
    clear.session_started(header(0));
    clear.fault_seen(header(1));
    EXPECT_TRUE(clear.take_clear());
    EXPECT_FALSE(clear.take_clear());
}

// requested restart, the old session faults on its way down after the grant changed and the M4
// restarts at once: the new session starts with the carried count and is running when the
// supervisor charges that fault
TEST(ScriptingM4NetRegionClear, FaultCarriedAcrossARequestedRestartDoesNotClear)
{
    NetRegionClear clear;
    clear.session_started(header(0));   // session A
    // the request's grant: A was running, no clear
    EXPECT_FALSE(clear.take_clear());
    clear.session_started(header(1));   // session B, header showing A's fault
    clear.fault_seen(header(1));        // check_running() charges A's fault while B runs
    EXPECT_FALSE(clear.take_clear());
    // B then faults itself
    clear.session_started(header(1));
    clear.fault_seen(header(2));
    EXPECT_TRUE(clear.take_clear());
}

// a fault record cleared at publish() reads as a lower count: not this session's fault
TEST(ScriptingM4NetRegionClear, LowerCountDoesNotClear)
{
    NetRegionClear clear;
    clear.session_started(header(3));
    clear.fault_seen(header(0));
    EXPECT_FALSE(clear.take_clear());
}

TEST(ScriptingM4NetRegionClear, ProcessExitClears)
{
    NetRegionClear clear;
    clear.session_started(header(5));
    clear.process_exited();
    EXPECT_TRUE(clear.take_clear());
    // a new session forgets an exit it did not see cleared
    clear.process_exited();
    clear.session_started(header(5));
    EXPECT_FALSE(clear.take_clear());
}

#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
// the carried fault: the grant leaves the region and its generation alone, as grant_restart() does
TEST(ScriptingM4NetRegionClear, CarriedFaultKeepsTheGeneration)
{
    static NetM4::Region region;
    std::atomic<uint32_t> generation {0};
    NetM4::reset_region(region, generation);   // start
    region.ports[0].tx_control.head.value.store(12);
    NetRegionClear clear;
    clear.session_started(header(0));
    clear.session_started(header(1));
    clear.fault_seen(header(1));
    if (clear.take_clear()) {
        NetM4::reset_region(region, generation);
    }
    EXPECT_EQ(generation.load(), 1U);
    EXPECT_EQ(region.ports[0].tx_control.head.value.load(), 12U);
    clear.fault_seen(header(2));
    if (clear.take_clear()) {
        NetM4::reset_region(region, generation);
    }
    EXPECT_EQ(generation.load(), 2U);
    EXPECT_EQ(region.ports[0].tx_control.head.value.load(), 0U);
}
#endif

AP_GTEST_MAIN()
