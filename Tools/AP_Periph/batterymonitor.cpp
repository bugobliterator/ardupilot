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

// #ifdef HAL_PERIPH_ENABLE_BATTMON

static bool turned_on = false;
static bool charging = false;
extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;
static uint32_t button_last_released_time;
static bool released;

#define POWER_ON_DELAY 1000 // Need to press for 1s to turn on
#define POWER_OFF_DELAY 2500 // Need to press for 5s to turn on

void AP_Periph_FW::battmon_init(void)
{
    hal.gpio->pinMode(HAL_GPIO_MODE, 0); // do input mode
    battmon.init();
}

void AP_Periph_FW::battmon_update(void)
{

    if (!charging) {
        if (!hal.gpio->read(HAL_GPIO_POWER_BUTTON)) {
            released = true;
            button_last_released_time = AP_HAL::millis();
        }
        
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
    }

    if (battmon.is_balancing()) {
        hal.gpio->write(HAL_GPIO_LED3, HAL_LED_ON);
    } else {
        hal.gpio->write(HAL_GPIO_LED3, !HAL_LED_ON);
    }

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