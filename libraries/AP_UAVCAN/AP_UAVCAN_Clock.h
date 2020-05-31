#pragma once

#include <uavcan/uavcan.hpp>

namespace uavcan
{
class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable
{
public:
    SystemClock() = default;

    void adjustUtc(uavcan::UtcDuration adjustment) override
    {
        utc_adjustment_usec = adjustment.toUSec();
    }

    uavcan::MonotonicTime getMonotonic() const override
    {
        return uavcan::MonotonicTime::fromUSec(AP_HAL::native_micros64());
    }

    uavcan::UtcTime getUtc() const override
    {
        return uavcan::UtcTime::fromUSec(AP_HAL::native_micros64() + utc_adjustment_usec);
    }

    int64_t getAdjustUsec() const
    {
        return utc_adjustment_usec;
    }

    static SystemClock& instance()
    {
        static SystemClock inst;
        return inst;
    }

private:
    int64_t utc_adjustment_usec;
};
}