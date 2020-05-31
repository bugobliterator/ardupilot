#pragma once

#include <AP_HAL/AP_HAL.h>
#if HAL_NUM_CAN_IFACES

#include <uavcan/uavcan.hpp>

namespace uavcan
{

class CanDriver;

class CanIface : public ICanIface, Noncopyable
{
    friend class CanIfaceMgr;
    AP_HAL::CANIface* can_iface_;
public:
    CanIface(AP_HAL::CANIface *can_iface) : can_iface_(can_iface) {}

    virtual int16_t send(const CanFrame& frame, MonotonicTime tx_deadline,
                         CanIOFlags flags) override;

    virtual int16_t receive(CanFrame& out_frame, MonotonicTime& out_ts_monotonic,
                            UtcTime& out_ts_utc, CanIOFlags& out_flags) override;

    int16_t configureFilters(const CanFilterConfig* filter_configs,
                             uint16_t num_configs) override;

    uint16_t getNumFilters() const override;

    uint64_t getErrorCount() const override;
};

/**
 * Generic CAN driver.
 */
class CanIfaceMgr : public ICanDriver, Noncopyable
{
    CanIface* ifaces[HAL_NUM_CAN_IFACES];
    uint8_t num_ifaces;
    HAL_EventHandle _event_handle;
public:
    bool add_interface(AP_HAL::CANIface *can_drv);

    ICanIface* getIface(uint8_t iface_index) override;

    uint8_t getNumIfaces() const override;

    CanSelectMasks makeSelectMasks(const CanSelectMasks in_mask, const CanFrame* (& pending_tx)[MaxCanIfaces]) const;

    int16_t select(CanSelectMasks& inout_masks,
                   const CanFrame* (& pending_tx)[MaxCanIfaces],
                   const MonotonicTime blocking_deadline) override;
};

}
#endif
