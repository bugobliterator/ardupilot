#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_CallStatus.h>

using namespace ModuleM4;

static BindingReplyHeader reply(ReplyStatus transport, int32_t status, uint16_t out_size)
{
    BindingReplyHeader r {};
    r.transport = uint8_t(transport);
    r.status = status;
    r.out_size = out_size;
    return r;
}

TEST(ModuleM4CallStatus, AdapterStatusPassesAndOutputCopiesOnlyOnOk)
{
    auto o = call_outcome(40, reply(ReplyStatus::Ok, STATUS_OK, 12), 12);
    EXPECT_EQ(o.status, STATUS_OK);
    EXPECT_TRUE(o.copy_output);
    o = call_outcome(28, reply(ReplyStatus::Ok, STATUS_UNAVAILABLE, 0), 12);
    EXPECT_EQ(o.status, STATUS_UNAVAILABLE);
    EXPECT_FALSE(o.copy_output);
}

TEST(ModuleM4CallStatus, DutyCapRefusalIsBusy)
{
    const auto o = call_outcome(28, reply(ReplyStatus::Ok, STATUS_BUSY, 0), 4);
    EXPECT_EQ(o.status, STATUS_BUSY);
    EXPECT_FALSE(o.copy_output);
}

TEST(ModuleM4CallStatus, TransportFailures)
{
    EXPECT_EQ(call_outcome(28, reply(ReplyStatus::UnknownMethod, 0, 0), 4).status, STATUS_INVALID);
    EXPECT_EQ(call_outcome(28, reply(ReplyStatus::Malformed, 0, 0), 4).status, STATUS_HOST_ERROR);
    EXPECT_EQ(call_outcome(-110, reply(ReplyStatus::Ok, STATUS_OK, 4), 4).status, STATUS_HOST_ERROR);  // -ETIMEDOUT
}

TEST(ModuleM4CallStatus, OutputSizeMismatchLeavesOutputUntouched)
{
    const auto o = call_outcome(36, reply(ReplyStatus::Ok, STATUS_OK, 8), 12);
    EXPECT_EQ(o.status, STATUS_HOST_ERROR);
    EXPECT_FALSE(o.copy_output);
}

TEST(ModuleM4CallStatus, UnknownStatusIsAHostError)
{
    EXPECT_EQ(call_outcome(28, reply(ReplyStatus::Ok, 7, 0), 0).status, STATUS_HOST_ERROR);
    EXPECT_EQ(call_outcome(28, reply(ReplyStatus::Ok, -6, 0), 0).status, STATUS_HOST_ERROR);
}

AP_GTEST_MAIN()
