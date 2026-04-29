#include "AC_CustomControl_Simulink.h"

#if AP_CUSTOMCONTROL_SIMULINK_ENABLED

const AP_Param::GroupInfo AC_CustomControl_Simulink::var_info[] = {
    AP_GROUPEND
};

AC_CustomControl_Simulink::AC_CustomControl_Simulink(AC_CustomControl& frontend, AP_AHRS_View*& ahrs,
                                                     AC_AttitudeControl*& att_control,
                                                     AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _simulink = AC_Simulink_Factory::createSimulinkInstance();
    if (_simulink != nullptr) {
        _simulink->init();
    }
}

Vector3f AC_CustomControl_Simulink::update()
{
    if (_simulink == nullptr) {
        return Vector3f{};
    }

    // run the Simulink model step. MW_setTorque / MW_setThrust callbacks stash
    // the latest body-frame torques and thrust on the AC_Simulink instance.
    return _simulink->update();
}

float AC_CustomControl_Simulink::get_thrust() const
{
    if (_simulink == nullptr) {
        return 0.0f;
    }
    return _simulink->get_thrust();
}

void AC_CustomControl_Simulink::reset()
{
    if (_simulink != nullptr) {
        _simulink->reset();
    }
}

#endif  // AP_CUSTOMCONTROL_SIMULINK_ENABLED
