
#include <AP_HAL/AP_HAL.h>
#include <AP_ONVIF/AP_ONVIF.h> 
// #include <DeviceBinding.nsmap>
// #include <MediaBinding.nsmap>
// #include <PTZBinding.nsmap>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_ONVIF onvif;

void setup()
{
    printf("AP_ONVIF library test\n");
    if (!onvif.init()) {
        printf("Failed to initialise onvif");
    }
}

void loop()
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
