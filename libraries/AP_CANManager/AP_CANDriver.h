#pragma once


#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_CANManager;
class AP_CANDriver
{
public:

    friend class AP_CANManager;
    
    // init method for protocol drivers, specify driver index and if filters
    // are to be enabled
    virtual void init(uint8_t driver_index, bool enable_filters) = 0;
    
    // link protocol drivers with interfaces by adding reference to CANIface
    virtual bool add_interface(AP_HAL::CANIface* can_iface) = 0;

};
