#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_M7Clock.h>

using namespace ModuleM4;

// the M7's view: micros() is the low 32 bits of micros64(), millis() is uint32_t(micros64() / 1000)
static uint32_t m7_micros(uint64_t micros64)
{
    return uint32_t(micros64);
}
static uint32_t m7_millis(uint64_t micros64)
{
    return uint32_t(micros64 / 1000U);
}

static const uint64_t WRAP = 1ULL << 32;

// calibrated from an M7 millis() read rpc_us before the counter, local clock in step
static M7Clock calibrated_at(uint64_t m7_us, uint64_t rpc_us, uint32_t local_ms)
{
    M7Clock clock {};
    m7_clock_calibrate(clock, m7_millis(m7_us), m7_micros(m7_us + rpc_us), local_ms);
    return clock;
}

TEST(ModuleM4M7Clock, CalibrationGivesTheM7Millis)
{
    const uint64_t times[] = {
        0, 999, 1000, 123456789, WRAP - 1, WRAP, WRAP + 1, 3 * WRAP + 17, 999 * WRAP + 5,
        1000 * WRAP - 1, 1000 * WRAP, 1000 * WRAP + 1, 2500 * WRAP + 42, 0xFFFFFFFFFFFULL,
    };
    const uint64_t delays[] = { 0, 1, 180, 999, 1000, 250000, 1000000, 1500000 };
    for (uint64_t t : times) {
        for (uint64_t delay : delays) {
            const M7Clock clock = calibrated_at(t, delay, 5);
            EXPECT_TRUE(clock.calibrated);
            EXPECT_EQ(m7_clock_millis(clock), m7_millis(t + delay)) << t << " " << delay;
            EXPECT_EQ(uint32_t(clock.micros64), m7_micros(t + delay));
        }
    }
}

// the M7's millis() is not micros() / 1000 once TIM5 has wrapped
TEST(ModuleM4M7Clock, MillisIsNotTheCounterOverAThousandAfterAWrap)
{
    const uint64_t t = WRAP + 5000;
    const M7Clock clock = calibrated_at(t, 0, 0);
    EXPECT_EQ(m7_clock_millis(clock), 4294972U);
    EXPECT_NE(m7_clock_millis(clock), m7_micros(t) / 1000U);
}

TEST(ModuleM4M7Clock, AdvanceFollowsCounterWraps)
{
    // reads every ~10 min, local ms clock in step with the M7, across several counter wraps
    uint64_t t = WRAP - 3000000;   // 3 s before the first wrap
    uint32_t local_ms = 4000000000U;   // near its own wrap too
    M7Clock clock = calibrated_at(t, 200, local_ms);
    for (int i = 0; i < 100; i++) {
        const uint64_t step = 600000000ULL + uint64_t(i) * 7919U;
        t += step;
        local_ms += uint32_t(step / 1000U);
        m7_clock_advance(clock, m7_micros(t), local_ms);
        EXPECT_EQ(m7_clock_millis(clock), m7_millis(t)) << i;
    }
}

TEST(ModuleM4M7Clock, AdvanceFindsMissedWraps)
{
    // gaps of one to five whole counter wraps plus a little, with the local clock off by +-20 s
    uint64_t t = 12345678;
    uint32_t local_ms = 777;
    M7Clock clock = calibrated_at(t, 0, local_ms);
    const int64_t skews_ms[] = { -20000, -1, 0, 1, 20000 };
    for (uint64_t wraps = 1; wraps <= 5; wraps++) {
        for (int64_t skew : skews_ms) {
            for (uint64_t extra : { uint64_t(0), uint64_t(1), uint64_t(WRAP / 3), uint64_t(WRAP - 1) }) {
                const uint64_t step = wraps * WRAP + extra;
                t += step;
                local_ms += uint32_t(int64_t(step / 1000U) + skew);
                m7_clock_advance(clock, m7_micros(t), local_ms);
                EXPECT_EQ(m7_clock_millis(clock), m7_millis(t)) << wraps << " " << skew << " " << extra;
            }
        }
    }
}

TEST(ModuleM4M7Clock, AdvanceWithASlowLocalClockAddsNoWrap)
{
    // the local clock reads a little less than the counter moved (whole ms, read before the counter)
    uint64_t t = 5 * WRAP + 100;
    uint32_t local_ms = 10;
    M7Clock clock = calibrated_at(t, 0, local_ms);
    t += 2500;
    local_ms += 1;
    m7_clock_advance(clock, m7_micros(t), local_ms);
    EXPECT_EQ(m7_clock_millis(clock), m7_millis(t));
    t += 999;
    m7_clock_advance(clock, m7_micros(t), local_ms);
    EXPECT_EQ(m7_clock_millis(clock), m7_millis(t));
}

TEST(ModuleM4M7Clock, MillisWrapsAtTheM7Uint32Wrap)
{
    // around the 49.7 day uint32_t millis() wrap
    uint64_t t = (1ULL << 32) * 1000U - 5000;
    uint32_t local_ms = 0;
    M7Clock clock = calibrated_at(t, 0, local_ms);
    EXPECT_EQ(m7_clock_millis(clock), 0xFFFFFFFBU);
    for (int i = 0; i < 10; i++) {
        t += 1000;
        local_ms += 1;
        m7_clock_advance(clock, m7_micros(t), local_ms);
        EXPECT_EQ(m7_clock_millis(clock), m7_millis(t)) << i;
        EXPECT_LT(clock.micros64, M7_MILLIS_PERIOD_US);
    }
    EXPECT_EQ(m7_clock_millis(clock), 5U);
}

TEST(ModuleM4M7Clock, RandomTimesAndSteps)
{
    uint64_t state = 0x9E3779B97F4A7C15ULL;
    auto next = [&state]() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    };
    for (int run = 0; run < 1000; run++) {
        uint64_t t = next() % (20000ULL * WRAP);
        uint32_t local_ms = uint32_t(next());
        M7Clock clock = calibrated_at(t, next() % 1000000U, local_ms);
        t = clock.micros64;   // the calibrated time, modulo the millis() period
        for (int i = 0; i < 50; i++) {
            const uint64_t step = next() % (3 * WRAP);
            t += step;
            local_ms += uint32_t(step / 1000U);
            m7_clock_advance(clock, m7_micros(t), local_ms);
            ASSERT_EQ(m7_clock_millis(clock), m7_millis(t)) << run << " " << i;
        }
    }
}

AP_GTEST_MAIN()
