#pragma once
/*
  M4 Lua build: the M7's custom rotations (ROTATION_CUSTOM_1/2) are parameters
  the M4 cannot read. Vector3::rotate and Quaternion::from_rotation (and so
  Quaternion::rotate) call these instead: the value is left unchanged and
  `refused` is set. The local binding layer clears `refused` before a call
  that takes a Rotation and raises a Lua error when it is set afterwards.
 */
#include <AP_Math/AP_Math.h>

#define AP_CUSTOMROTATIONS_ENABLED 1

class AP_CustomRotations {
public:
    bool refused;

    template <typename T>
    void rotate(enum Rotation, Vector3<T> &) { refused = true; }
    template <typename T>
    void from_rotation(enum Rotation, QuaternionT<T> &) { refused = true; }
};

namespace AP {
// zero-initialised, without a constructor: in the Lua RAM .bss of the object that uses it
inline AP_CustomRotations &custom_rotations()
{
    static AP_CustomRotations custom_rotations;
    return custom_rotations;
}
}
