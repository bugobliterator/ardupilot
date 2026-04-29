#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_SIMULINK_ENABLED

#include "AC_CustomControl_Backend.h"
#include <AC_Simulink/AC_Simulink_Factory.h>

class AC_CustomControl_Simulink : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Simulink(AC_CustomControl& frontend, AP_AHRS_View*& ahrs,
                              AC_AttitudeControl*& att_control,
                              AP_MotorsMulticopter*& motors, float dt);

    Vector3f update() override;
    float get_thrust() const override;
    void reset() override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AC_Simulink_Base *_simulink;
};

#endif  // AP_CUSTOMCONTROL_SIMULINK_ENABLED
