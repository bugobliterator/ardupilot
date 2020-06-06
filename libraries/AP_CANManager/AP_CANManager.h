#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_NUM_CAN_IFACES

#include <AP_Param/AP_Param.h>
#include "AP_SLCANIface.h"
#include "AP_CANDriver.h"
#ifndef MAX_NUMBER_OF_CAN_DRIVERS
#define MAX_NUMBER_OF_CAN_DRIVERS HAL_NUM_CAN_IFACES
#endif

class AP_CANManager
{
public:
    AP_CANManager();

    /* Do not allow copies */
    AP_CANManager(const AP_CANManager &other) = delete;
    AP_CANManager &operator=(const AP_CANManager&) = delete;

    static AP_CANManager* get_singleton()
    {
        return _singleton;
    }

    enum LogLevel {
        LOG_NONE,
        LOG_ERROR,
        LOG_WARNING,
        LOG_INFO,
        LOG_DEBUG,
    };

    enum Driver_Type : uint8_t {
        Driver_Type_None = 0,
        Driver_Type_UAVCAN = 1,
        Driver_Type_KDECAN = 2,
        Driver_Type_ToshibaCAN = 3,
        Driver_Type_PiccoloCAN = 4,
        Driver_Type_TestCAN = 5,
    };

    void init(void);

    // returns number of active CAN Drivers
    uint8_t get_num_drivers(void) const
    {
        return _num_drivers;
    }

    // return driver for index i
    AP_CANDriver* get_driver(uint8_t i) const
    {
        if (i < HAL_NUM_CAN_IFACES) {
            return _drivers[i];
        }
        return nullptr;
    }

    // Method to log status and debug information for review while debugging
    void log_text(AP_CANManager::LogLevel loglevel, const char *tag, const char *fmt, ...);
    
    uint32_t log_retrieve(char* data, uint32_t max_size) const;

    // return driver type index i
    Driver_Type get_driver_type(uint8_t i) const
    {
        if (i < HAL_NUM_CAN_IFACES) {
            return _driver_type_cache[i];
        }
        return Driver_Type_None;
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    
    // Parameter interface for CANIfaces
    class CANIface_Params
    {
        friend class AP_CANManager;

    public:
        CANIface_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _driver_number;
        uint8_t _driver_number_cache;
        AP_Int32 _bitrate;
    };

    //Parameter Interface for CANDrivers
    class CANDriver_Params
    {
        friend class AP_CANManager;

    public:
        CANDriver_Params()
        {
            AP_Param::setup_object_defaults(this, var_info);
        }
        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_Int8 _driver_type;
        AP_CANDriver* _testcan;
        AP_CANDriver* _uavcan;
        AP_CANDriver* _kdecan;
    };

    CANIface_Params _interfaces[HAL_NUM_CAN_IFACES];
    AP_CANDriver* _drivers[MAX_NUMBER_OF_CAN_DRIVERS] {};
    CANDriver_Params _drv_param[MAX_NUMBER_OF_CAN_DRIVERS];
    Driver_Type _driver_type_cache[MAX_NUMBER_OF_CAN_DRIVERS];

    AP_Int8 _loglevel;
    uint8_t _num_drivers;
    SLCAN::CANIface _slcan_interface;
    static AP_CANManager *_singleton;

    char* _log_buf;
    uint32_t _log_pos;
};

namespace AP
{
AP_CANManager& can();
}

#endif
