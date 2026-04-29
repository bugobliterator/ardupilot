#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

class AC_Simulink_Base {
public:
    virtual ~AC_Simulink_Base() {}
    virtual void init() = 0;
    virtual Vector3f update() = 0;
    virtual void reset() = 0;
    virtual float get_thrust() const { return 0.0f; }
};
