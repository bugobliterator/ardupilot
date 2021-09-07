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

#include "AP_Periph.h"
#include "hal.h"
#include <drivers/stm32/canard_stm32.h>
#include <AP_BattMonitor/AP_BattMonitor_SMBus.h>
#include <AP_HAL_ChibiOS/I2CSlave.h>
// #ifdef HAL_PERIPH_ENABLE_BATTMON

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

static bool charging = false;

#ifdef HAL_GPIO_POWER_BUTTON
static bool turned_on = false;
static uint32_t button_last_released_time;
static bool released;
#endif

#define POWER_ON_DELAY 1000 // Need to press for 1s to turn on
#define POWER_OFF_DELAY 2500 // Need to press for 5s to turn on

#ifdef HAL_I2C_SLAVE_LIST
#define BATTMONITOR_SMBUS_SOLO_CELL_VOLTAGE         0x28    // cell voltage register
#define BATTMONITOR_SMBUS_SOLO_CURRENT              0x2a    // current register
#define BATTMONITOR_SMBUS_TEMP                      0x08    // Temperature
// #define BATTMONITOR_SMBUS_VOLTAGE                   0x09    // Voltage
// #define BATTMONITOR_SMBUS_CURRENT                   0x0A    // Current
#define BATTMONITOR_SMBUS_REMAINING_CAPACITY        0x0F    // Remaining Capacity
#define BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY      0x10    // Full Charge Capacity
#define BATTMONITOR_SMBUS_CYCLE_COUNT               0x17    // Cycle Count
#define BATTMONITOR_SMBUS_SPECIFICATION_INFO        0x1A    // Specification Info
#define BATTMONITOR_SMBUS_SERIAL                    0x1C    // Serial Number
#define BATTMONITOR_SMBUS_MANUFACTURE_NAME          0x20    // Manufacture Name
#define BATTMONITOR_SMBUS_MANUFACTURE_DATA          0x23    // Manufacture Data
#define BATTMONITOR_SMBUS_NONE                      0x00    // Empty Dummy


// // Specification Register Block
// static struct reg_block {
//     uint8_t reg;
//     uint16_t data;
// }reg_blocks[] = {
//     {BATTMONITOR_SMBUS_SOLO_CELL_VOLTAGE, 0},
//     {BATTMONITOR_SMBUS_SOLO_CURRENT, 0},
//     {BATTMONITOR_SMBUS_TEMP, 0},
//     {BATTMONITOR_SMBUS_REMAINING_CAPACITY, 0},
//     {BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY, 0},
//     {BATTMONITOR_SMBUS_CYCLE_COUNT, 0},
//     {BATTMONITOR_SMBUS_SPECIFICATION_INFO, 0},
//     {BATTMONITOR_SMBUS_SERIAL, 0},
//     {BATTMONITOR_SMBUS_MANUFACTURE_NAME, 0},
//     {BATTMONITOR_SMBUS_MANUFACTURE_DATA, 0},
//     {BATTMONITOR_SMBUS_NONE, 0},
// };
static const char mfg_name[] = AP_BATT_MONITOR_MANUFACTURER_NAME;
static const char device_name[] = AP_BATT_MONITOR_DEVICE_NAME;

static struct  BattMonI2CIface {
    AP_BattMonitor *battmon;
    ChibiOS::I2CSlave slv;
    void rx_handle(uint8_t trx_idx, uint8_t recv_byte);
    uint8_t tx_handle(void);
    uint8_t tx_byte_idx;
    uint8_t current_request_base_address;

    // Params
    const uint16_t version = (0x1U) | (0x2U << 4);

    uint16_t cell_voltage[AP_BATT_MONITOR_CELLS_MAX];
    uint32_t current_mA;
    struct reg_block* get_reg_block(uint8_t reg);
    uint8_t send_buf[2*AP_BATT_MONITOR_CELLS_MAX + 2]; // 1B Length + Max 8 bytes + 1 PEC
    uint8_t send_length;
} battmon_i2c;

void BattMonI2CIface::rx_handle(uint8_t trx_idx, uint8_t recv_byte)
{
    if (trx_idx == 0) {
        tx_byte_idx = 0;
        current_request_base_address = recv_byte & ~(uint8_t(1)<<7);
    } else {
        // nothing to do
        return;
    }

    memset(send_buf, 0, sizeof(send_buf));
    send_length = 0;
    switch (current_request_base_address) {
        case BATTMONITOR_SMBUS_SOLO_CURRENT:
        {
            float current_amps;
            if (!battmon->current_amps(current_amps)) {
                current_mA = 0;
                break;
            }
            current_mA = ((uint32_t)(current_amps*1000.0f));
            for (uint8_t i = 0; i < sizeof(current_mA); i++) {
                send_buf[i+1] = GET_BYTE_BY_INDEX(current_mA, i);
            }
            send_length = sizeof(current_mA) + 2;
            break;
        }
        case BATTMONITOR_SMBUS_SOLO_CELL_VOLTAGE:
        {
            for (uint8_t i = 0; i < AP_BATT_MONITOR_CELLS_MAX; i++) {
                cell_voltage[i] = battmon->get_cell_voltages().cells[i];
            }
            for (uint8_t i=0; i < 2*AP_BATT_MONITOR_CELLS_MAX; i++) {
                send_buf[i+1] = GET_BYTE_BY_INDEX(cell_voltage[i/2], i%2);
            }
            send_length = 2*AP_BATT_MONITOR_CELLS_MAX + 2;
            break;
        }
        case BATTMONITOR_SMBUS_TEMP:
        {// = 0x08,                 // Temperature
            float temperature;
            if (!battmon->get_temperature(temperature)) {
                temperature = 0.0f;
            }
            uint16_t temperature_send = (uint16_t)((temperature + 273.1f) * 10.0f);
            send_buf[0] = GET_BYTE_BY_INDEX(temperature_send, 0);
            send_buf[1] = GET_BYTE_BY_INDEX(temperature_send, 1);
            send_length = 3;
            break;
        }
        case BATTMONITOR_SMBUS_REMAINING_CAPACITY:
        {// = 0x0F,   // Remaining Capacity
            float consumed_mah;
            if (!battmon->consumed_mah(consumed_mah)) {
                consumed_mah = 0.0f;
            }
            uint16_t consumed_mah_send = (uint16_t)(battmon->pack_capacity_mah() - (uint16_t)consumed_mah);
            send_buf[0] = GET_BYTE_BY_INDEX(consumed_mah_send, 0);
            send_buf[1] = GET_BYTE_BY_INDEX(consumed_mah_send, 1);
            send_length = 3;
            break;        
        }
        case BATTMONITOR_SMBUS_FULL_CHARGE_CAPACITY:
        {// = 0x10, // Full Charge Capacity
            uint16_t pack_capacity_mah = (uint16_t)battmon->pack_capacity_mah();
            send_buf[0] = GET_BYTE_BY_INDEX(pack_capacity_mah, 0);
            send_buf[1] = GET_BYTE_BY_INDEX(pack_capacity_mah, 1);
            send_length = 3;
            break;
        }
        case BATTMONITOR_SMBUS_SPECIFICATION_INFO:
        {// = 0x1A,   // Specification Info
            send_buf[0] = GET_BYTE_BY_INDEX(version, 0);
            send_buf[1] = GET_BYTE_BY_INDEX(version, 1);
            send_length = 3;
            break;
        }
        case BATTMONITOR_SMBUS_SERIAL:
        { // = 0x1C,               // Serial Number
            send_buf[0] = GET_BYTE_BY_INDEX(APJ_BOARD_ID, 0);
            send_buf[1] = GET_BYTE_BY_INDEX(APJ_BOARD_ID, 1);
            send_length = 3;
            break;
        }
        case BATTMONITOR_SMBUS_MANUFACTURE_NAME:
        { // = 0x20,     // Manufacture Name
            send_length = (sizeof(send_buf) < sizeof(mfg_name))?sizeof(send_buf):sizeof(mfg_name);
            memcpy(send_buf, mfg_name, send_length);
            break;
        }
        case BATTMONITOR_SMBUS_MANUFACTURE_DATA:
        { // = 0x23,     // Manufacture Data
            send_length = (sizeof(send_buf) < sizeof(device_name))?sizeof(send_buf):sizeof(device_name);
            memcpy(send_buf, device_name, send_length);
            send_length += 2;
            break;
        }
        case BATTMONITOR_SMBUS_CYCLE_COUNT:
        {    // = 0x17,          // Cycle Count
            // TODO
            send_length = 3;
            break;
        }
        default:
            // return error
            break;
    }
    if (send_length > 3) { // Block transfer
        send_buf[0] = send_length-2; // Exclude data length and PEC from the data size
        send_buf[send_length-1] = AP_BattMonitor_SMBus::get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, current_request_base_address, 
                                        true, send_buf, send_length - 1);
    } else if (send_length == 3) { // word transfer
        send_buf[2] =  AP_BattMonitor_SMBus::get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, current_request_base_address, 
                                        true, send_buf, 2);
    }
}

uint8_t BattMonI2CIface::tx_handle()
{
    tx_byte_idx++;
    if (tx_byte_idx > send_length) {
        return 0;
    }
    return send_buf[tx_byte_idx-1];
}
#endif

void AP_Periph_FW::battmon_init(void)
{
    hal.gpio->pinMode(HAL_GPIO_MODE, 0); // do input mode
    battmon.init();
#ifdef HAL_I2C_SLAVE_LIST
    battmon_i2c.battmon = &battmon;
    battmon_i2c.slv.init(0, AP_BATTMONITOR_SMBUS_I2C_ADDR, 
                100000,
                FUNCTOR_BIND(&battmon_i2c, &BattMonI2CIface::rx_handle, void, uint8_t, uint8_t),
                FUNCTOR_BIND(&battmon_i2c, &BattMonI2CIface::tx_handle, uint8_t));
#endif
}

void AP_Periph_FW::battmon_update(void)
{

    if (!charging) {
#ifdef HAL_GPIO_POWER_BUTTON
        if (!hal.gpio->read(HAL_GPIO_POWER_BUTTON)) {
            released = true;
            button_last_released_time = AP_HAL::millis();
        }
#endif
        
#ifdef HAL_GPIO_LED1
        if ((AP_HAL::millis() - button_last_released_time > POWER_ON_DELAY) && !turned_on && released) {
            turned_on = true;
            hal.gpio->write(HAL_GPIO_LED1, HAL_LED_ON);
            battmon.power_on();
            released = false;
        } else if ((AP_HAL::millis() - button_last_released_time > POWER_OFF_DELAY) && turned_on && released) {
            turned_on = false;
            hal.gpio->write(HAL_GPIO_LED1, !HAL_LED_ON);
            battmon.power_off();
            released = false;
        }
#endif

#ifdef HAL_GPIO_CHARGING
        if (hal.gpio->read(HAL_GPIO_CHARGING)) {
            if (!turned_on) {
                // lets begin Charging
                battmon.start_charging();
                charging = true;
                hal.gpio->write(HAL_GPIO_LED2, HAL_LED_ON);
            }
        }
    } else {
        if (!hal.gpio->read(HAL_GPIO_CHARGING)) {
            // stop Charging
            battmon.stop_charging();
            charging = false;
            hal.gpio->write(HAL_GPIO_LED2, !HAL_LED_ON);
        }
#endif
    }

#ifdef HAL_GPIO_LED3
    if (battmon.is_balancing()) {
        hal.gpio->write(HAL_GPIO_LED3, HAL_LED_ON);
    } else {
        hal.gpio->write(HAL_GPIO_LED3, !HAL_LED_ON);
    }
#endif

    battmon_cansend();
}

void AP_Periph_FW::battmon_1hz_update()
{
    can_printf("Cell Voltages: %f %f %f %f", battmon.get_cell_voltages().cells[0]/1000.f, 
                                             battmon.get_cell_voltages().cells[1]/1000.f,
                                             battmon.get_cell_voltages().cells[2]/1000.f,
                                             battmon.get_cell_voltages().cells[3]/1000.f);
}

// #endif