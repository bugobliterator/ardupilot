/* Copyright 2025 The MathWorks, Inc. */

#pragma once

/// @file    AC_Simulink_Normal.h
/// @brief   Normal mode code-deployment of Simulink

#include "AC_Simulink_Base.h"
#include "AC_Simulink_ModelDefines.h"

class AC_Simulink_Normal : public AC_Simulink_Base {
public:
    AC_Simulink_Normal();
    ~AC_Simulink_Normal() override {}

    void init() override;
    Vector3f update() override;
    void reset() override;

    void setErrorStatus(const char_T*);
    bool getErrorStatus();

    static AC_Simulink_Normal* get_singleton() { return _singleton; }

    // setters used by MW_setTorque / MW_setThrust callbacks from the generated model
    void set_torque(float roll, float pitch, float yaw) {
        _torque.x = roll;
        _torque.y = pitch;
        _torque.z = yaw;
    }
    void set_thrust(float thrust) { _thrust = thrust; }

    const Vector3f& get_torque() const { return _torque; }
    float get_thrust() const override { return _thrust; }

    CLASS_NO_COPY(AC_Simulink_Normal);

protected:
    Vector3f _torque;
    float    _thrust;

private:
    static AC_Simulink_Normal* _singleton;
};

using AC_Simulink = AC_Simulink_Normal;
