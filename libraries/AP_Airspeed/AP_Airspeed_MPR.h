#pragma once

#include "AP_Airspeed_Backend.h"

#if defined(AP_AIRSPEED_MPR_ENABLED) && AP_AIRSPEED_MPR_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#define MBAR_TO_PASCAL 100.0f


#define TRANSFER_FUNCTION_A_MIN ((1<<24)*0.1f)
#define TRANSFER_FUNCTION_A_MAX ((1<<24)*0.9f)


class AP_Airspeed_MPR : public AP_Airspeed_Backend
{
public:
    enum class MPR_TYPE {
        MPRSS0250MG0000SA = 0,
    };

    static AP_Airspeed_Backend *probe(AP_Airspeed &airspeed, uint8_t _instance, AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev1, enum MPR_TYPE mpr_type);

    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &_pressure) override { return false; }

    // return the current temperature in degrees C, if available
    bool get_temperature(float &_temperature) override { return false; }

private:

    enum class MPR_READ_STATE {
        IDLE,
        TRIGGERED,
        READ,
    };

    // Conversion function is as follows:
    //  ((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
    struct MPR_CONV {
        float pmin;
        float pmax;
        float outputmin;
        float outputmax;
        constexpr float toPascal(uint32_t press_counts) const {
            return ((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin) + pmin;
        }
    };

    static const MPR_CONV conv[];

    // temperature in degrees C
    static constexpr float toCelsius(uint32_t temp_counts) {
        return ((temp_counts * 200) / 16777215) - 50.0f;
    }

    AP_Airspeed_MPR(AP_Airspeed &airspeed, uint8_t instance, enum MPR_TYPE mpr_type);

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    enum MPR_TYPE _mpr_type;

    float pressure; // in pascal
    float temperature; // in celsius

    AP_HAL::Device::PeriodicHandle periodic_handle;
    bool _trigger_read();
    bool _check_read_triggered();
    void _read();
    MPR_READ_STATE read_state;
    uint32_t last_read_ms = 0;

};

#endif  // AP_AIRSPEED_MPR_ENABLED
