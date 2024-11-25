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
  backend driver for airspeed from Honeywell MPR series sensors
 */

#include "AP_Airspeed_MPR.h"
#include <GCS_MAVLink/GCS.h>
#ifdef HAL_GPIO_PIN_MPR_RESET
#include "hal.h"
#endif
#if defined(AP_AIRSPEED_MPR_ENABLED) && AP_AIRSPEED_MPR_ENABLED

extern const AP_HAL::HAL& hal;

const AP_Airspeed_MPR::MPR_CONV AP_Airspeed_MPR::conv[] = {
    {.pmin = 0.0f, .pmax = 250.0f * MBAR_TO_PASCAL, .outputmin = TRANSFER_FUNCTION_A_MIN, .outputmax = TRANSFER_FUNCTION_A_MAX}, // MPRSS0250MG0000SA
};

AP_Airspeed_MPR::AP_Airspeed_MPR(AP_Airspeed &airspeed, uint8_t _instance, enum MPR_TYPE mpr_type) :
    AP_Airspeed_Backend(airspeed, _instance),
    _mpr_type(mpr_type)
{}


AP_Airspeed_Backend *AP_Airspeed_MPR::probe(AP_Airspeed &airspeed, uint8_t _instance, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev, enum MPR_TYPE mpr_type)
{
    if (!dev) {
        return nullptr;
    }
    AP_Airspeed_MPR *sensor = new AP_Airspeed_MPR(airspeed, _instance, mpr_type);
    if (!sensor) {
        return nullptr;
    }
    sensor->_dev = std::move(dev);
    // we don't call init here, add_backend will call it for us
    return sensor;
}

bool AP_Airspeed_MPR::init() {
    WITH_SEMAPHORE(_dev->get_semaphore());
    // send 0xAA to the sensor to read status
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    bool healthy;
    uint8_t retries = 10;
    uint8_t cmd[] = {0xAA, 0x00, 0x00};
    uint8_t data[3] = {};
    do {
        healthy = true;
        if (!_dev->transfer_fullduplex(cmd, data, 3)) {
            healthy = false;
            continue;
        }
        // check the device is powered up
        if ((data[0] != 0x40) && (data[0] != 0x60)) {
            healthy = false;
        }
        hal.scheduler->delay(1);
    } while (--retries && !healthy);
    
    if (!healthy) {
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MPR sensor found");
    // we have to wait for conversion to happen, which takes atleast 5ms, so we will poll every 6ms 
    periodic_handle = _dev->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&AP_Airspeed_MPR::_timer, void));
    return true;
}

bool AP_Airspeed_MPR::_trigger_read()
{
    uint8_t cmd[3] = {};
    uint8_t data[3] = {};
    cmd[0] = 0xAA; // trigger a new read
    if (!_dev->transfer_fullduplex(cmd, data, 3)) {
        return false;
    }
    return true;
}

bool AP_Airspeed_MPR::_check_read_triggered()
{
    uint8_t cmd[3] = {};
    uint8_t data[3] = {};
    cmd[0] = 0xF0; // trigger a new read
    if (!_dev->transfer_fullduplex(cmd, data, 3)) {
        return false;
    }
    return data[0] == 0x60;
}

void AP_Airspeed_MPR::_read() {
    uint8_t cmd[7] = {};
    uint8_t data[7] = {};

    cmd[0] = 0xF0; // read sensor data from last conversion
    if (!_dev->transfer_fullduplex(cmd, data, 7)) {
        return;
    }
    
    if (data[0] == 0x40) {
        _dev->adjust_periodic_callback(periodic_handle, 20000);
        pressure = conv[(int)_mpr_type].toPascal((data[1] << 16) | (data[2] << 8) | data[3]);
        temperature = toCelsius((data[4] << 16) | (data[5] << 8) | data[6]);
        // new data available
        // print data
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MPR[%d] [%u]: %02x %02x %02x %02x %02x %02x %02x", get_instance(), AP_HAL::millis() - last_read_ms, data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "MPR[%d] Pressure: %f Pa, Temp: %f C", get_instance(), pressure, temperature);
    }
}

void AP_Airspeed_MPR::_timer() {

    WITH_SEMAPHORE(_dev->get_semaphore());

    switch (read_state) {
        case MPR_READ_STATE::IDLE:
            // trigger a read
            read_state = _trigger_read()?MPR_READ_STATE::TRIGGERED:MPR_READ_STATE::IDLE;
            // we will wait for 1ms before checking if the read is triggered
            _dev->adjust_periodic_callback(periodic_handle, 1000);
            break;
        case MPR_READ_STATE::TRIGGERED:
            // check busy status, if not busy, it means the read wasn't triggered
            read_state = _check_read_triggered()?MPR_READ_STATE::READ:MPR_READ_STATE::IDLE;
            // we will wait for another 5ms before reading the data, giving the sensor time to convert
            _dev->adjust_periodic_callback(periodic_handle, 5000);
            break;
        case MPR_READ_STATE::READ:
            _read();
            read_state = MPR_READ_STATE::IDLE;
            // we will wait for 20ms before triggering the next read
            _dev->adjust_periodic_callback(periodic_handle, 20000);
            break;
    }

    if (read_state == MPR_READ_STATE::IDLE) {
        // re-adjust the timer to 20ms, in case we are not in the read state
        _dev->adjust_periodic_callback(periodic_handle, 20000);
    }

    last_read_ms = AP_HAL::millis();

}

#endif