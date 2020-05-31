/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 *   AP_CANManager - board specific configuration for CAN interface
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "AP_CANManager.h"

#if HAL_NUM_CAN_IFACES

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_KDECAN/AP_KDECAN.h>
#include <AP_ToshibaCAN/AP_ToshibaCAN.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#include "AP_CANTester.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/CANIface.h>
#endif

#define LOG_TAG "CANMGR"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_CANManager::var_info[] = {

#if HAL_NUM_CAN_IFACES > 0
    // @Group: P1_
    // @Path: ../AP_CANManager/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[0], "P1_", 1, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_NUM_CAN_IFACES > 1
    // @Group: P2_
    // @Path: ../AP_CANManager/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[1], "P2_", 2, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if HAL_NUM_CAN_IFACES > 2
    // @Group: P3_
    // @Path: ../AP_CANManager/canbus_interface.cpp
    AP_SUBGROUPINFO(_interfaces[2], "P3_", 3, AP_CANManager, AP_CANManager::CANIface_Params),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 0
    // @Group: D1_
    // @Path: ../AP_CANManager/canbus_driver.cpp
    AP_SUBGROUPINFO(_drv_param[0], "D1_", 4, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 1
    // @Group: D2_
    // @Path: ../AP_CANManager/canbus_driver.cpp
    AP_SUBGROUPINFO(_drv_param[1], "D2_", 5, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

#if MAX_NUMBER_OF_CAN_DRIVERS > 2
    // @Group: D3_
    // @Path: ../AP_CANManager/canbus_driver.cpp
    AP_SUBGROUPINFO(_drv_param[2], "D3_", 6, AP_CANManager, AP_CANManager::CANDriver_Params),
#endif

    // @Group: SLCAN_
    // @Path: ../AP_CANManager/canbus_driver.cpp
    AP_SUBGROUPINFO(_slcan_interface, "SLCAN_", 7, AP_CANManager, SLCAN::CANIface),

    // @Param: LOGLEVEL
    // @DisplayName: Loglevel
    // @Description: Loglevel for recording initialisation and debug information from CAN Interface
    // @Range: 0 4
    // @Values: 0: Log None, 1: Log Error, 2: Log Warning and below, 3: Log Info and below, 4: Log Everything
    // @User: Advanced
    AP_GROUPINFO("LOGLEVEL", 8, AP_CANManager, _loglevel, AP_CANManager::LOG_NONE),

    AP_GROUPEND
};

AP_CANManager *AP_CANManager::_singleton;

#define LOG_BUFFER_SIZE 1024

AP_CANManager::AP_CANManager()
{
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_CANManager must be singleton");
    }
    _singleton = this;
}

void AP_CANManager::init()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (AP::sitl() != nullptr) {
        if (AP::sitl()->speedup > 1) {
            log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "CAN is not supported under speedup.");

            return;
        }
    } else{
        AP_HAL::panic("CANManager: SITL not initialised!");
    }
#endif
    // We only allocate log buffer only when under debug
    if (_loglevel != AP_CANManager::LOG_NONE) {
        _log_buf = new char[LOG_BUFFER_SIZE];
        _log_pos = 0;
    }

    //Reset all SLCAN related params that needs resetting at boot
    _slcan_interface.reset_params();

    uint8_t drv_num;

    // loop through interfaces and allocate and initialise Iface,
    // Also allocate Driver objects, and add interfaces to them
    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
        // Get associated Driver to the interface
        drv_num = _interfaces[i]._driver_number_cache = _interfaces[i]._driver_number;
        if (drv_num == 0) {
            continue;
        }
        drv_num -= 1;

        if (hal.can[i] == nullptr) {
            // So if this interface is not allocated allocate it here, 
            // also pass the index of the CANBus
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
            const_cast <AP_HAL::HAL&> (hal).can[i] = new Linux::CANIface(i);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
            const_cast <AP_HAL::HAL&> (hal).can[i] = new HALSITL::CANIface(i);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
            const_cast <AP_HAL::HAL&> (hal).can[i] = new ChibiOS::CANIface(i);
#endif
        }

        // Initialise the interface we just allocated
        if (hal.can[i] != nullptr) {
            hal.can[i]->init(_interfaces[i]._bitrate, AP_HAL::CANIface::NormalMode);
        } else {
            continue;
        }

        // Check if this interface need hooking up to slcan passthrough
        // instead of a driver
        if (_slcan_interface.init_passthrough(i)) {
            // We will not be running any driver on this port,
            // we just do a passthrough with SLCAN
            continue;
        }

        // Find the driver type that we need to allocate and register this interface with
        Driver_Type drv_type = _driver_type_cache[drv_num] = (Driver_Type) _drv_param[drv_num]._driver_type.get();

        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "CAN Interface %d initialized well\n\r", i + 1);

        if (_drivers[drv_num] != nullptr) {
            //We already initialised the driver just add interface and move on
            log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d\n\r", i + 1, drv_num + 1);
            _drivers[drv_num]->add_interface(hal.can[i]);
            continue;
        }

        _num_drivers++;
        if (_num_drivers > MAX_NUMBER_OF_CAN_DRIVERS) {
            // We are exceeding number of drivers, 
            // this can't be happening time to panic
            AP_BoardConfig::config_error("Max number of CAN Drivers exceeded\n\r");
        }

        // Allocate the set type of Driver
        if (drv_type == Driver_Type_UAVCAN) {
            _drivers[drv_num] = _drv_param[drv_num]._uavcan = new AP_UAVCAN;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::config_error("Failed to allocate uavcan %d\n\r", i + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_UAVCAN*)_drivers[drv_num], AP_UAVCAN::var_info);
        } else if (drv_type == Driver_Type_KDECAN) {
            // To be replaced with macro saying if KDECAN library is included
            _drivers[drv_num] = _drv_param[drv_num]._kdecan = new AP_KDECAN;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::config_error("Failed to allocate KDECAN %d\n\r", drv_num + 1);
                continue;
            }

            AP_Param::load_object_from_eeprom((AP_KDECAN*)_drivers[drv_num], AP_KDECAN::var_info);
        } else if (drv_type == Driver_Type_ToshibaCAN) {
            _drivers[drv_num] = new AP_ToshibaCAN;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::config_error("Failed to allocate ToshibaCAN %d\n\r", drv_num + 1);
                continue;
            }
        } else if (drv_type == Driver_Type_PiccoloCAN) {
#if HAL_PICCOLO_CAN_ENABLE
            _drivers[drv_num] = new AP_PiccoloCAN;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::config_error("Failed to allocate PiccoloCAN %d\n\r", drv_num + 1);
                continue;
            }
#endif
        } else if (drv_type == Driver_Type_TestCAN) {
#if HAL_NUM_CAN_IFACES > 1
            _drivers[drv_num] = _drv_param[drv_num]._testcan = new CANTester;

            if (_drivers[drv_num] == nullptr) {
                AP_BoardConfig::config_error("Failed to allocate ToshibaCAN %d\n\r", drv_num + 1);
                continue;
            }
            AP_Param::load_object_from_eeprom((CANTester*)_drivers[drv_num], CANTester::var_info);
#endif
        } else {
            continue;
        }

        // Hook this interface to the selected Driver Type
        _drivers[drv_num]->add_interface(hal.can[i]);
        log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Adding Interface %d to Driver %d\n\r", i + 1, drv_num + 1);
        
    }

    for(drv_num = 0; drv_num < MAX_NUMBER_OF_CAN_DRIVERS; drv_num++) {
        //initialise all the Drivers
        if (_drivers[drv_num] == nullptr) {
            continue;
        }
        _drivers[drv_num]->init(drv_num, true);
    }

    // param count could have changed
    AP_Param::invalidate_count();
}

// Method used by CAN related library methods to report status and debug info
// The result of this method can be accessed via ftp get @SYS/can_log.txt
void AP_CANManager::log_text(AP_CANManager::LogLevel loglevel, const char *tag, const char *fmt, ...)
{
    if (_log_buf == nullptr) {
        return;
    }
    if ((loglevel > _loglevel) && (_loglevel != AP_CANManager::LOG_NONE)) {
        return;
    }

    //Tag Log Message
    switch (loglevel) {
    case AP_CANManager::LOG_DEBUG :
        _log_pos += hal.util->snprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, "\n%s DEBUG :", tag);
        break;

    case AP_CANManager::LOG_INFO :
        _log_pos += hal.util->snprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, "\n%s INFO :", tag);
        break;

    case AP_CANManager::LOG_WARNING :
        _log_pos += hal.util->snprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, "\n%s WARN :", tag);
        break;

    case AP_CANManager::LOG_ERROR :
        _log_pos += hal.util->snprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, "\n%s ERROR :", tag);
        break;

    default :
        return;
    }

    _log_pos %= LOG_BUFFER_SIZE;
    va_list arg_list;
    va_start(arg_list, fmt);
    _log_pos += hal.util->vsnprintf(&_log_buf[_log_pos], LOG_BUFFER_SIZE - _log_pos, fmt, arg_list);
    va_end(arg_list);
    _log_pos %= LOG_BUFFER_SIZE;
}

// log retrieve method used by file sys method to report can log
uint32_t AP_CANManager::log_retrieve(char* data, uint32_t max_size) const
{
    if (_loglevel == AP_CANManager::LOG_NONE || _log_buf == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Log buffer not available");
        return 0;
    }
    uint32_t read_len = MIN(max_size, _log_pos);
    memcpy(data, _log_buf, read_len);
    return read_len;
}

AP_CANManager& AP::can()
{
    return *AP_CANManager::get_singleton();
}

#endif

