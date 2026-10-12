#pragma once
/*
  M4 Lua build: the M7's custom rotations (ROTATION_CUSTOM_1/2) are parameters
  the M4 cannot read. Vector3::rotate and Quaternion::from_rotation (and so
  Quaternion::rotate) compile against these stubs, which leave the value
  unchanged. They are never reached: gen-bindings keeps every method with a
  Rotation argument remote (remote_method_takes_rotation), and the M4 build
  fails if the image contains them (STATE_READING_SYMBOLS in chibios/Makefile).
 */
#include <AP_Math/AP_Math.h>

#define AP_CUSTOMROTATIONS_ENABLED 1

class AP_CustomRotations {
public:
    template <typename T>
    void rotate(enum Rotation, Vector3<T> &) {}
    template <typename T>
    void from_rotation(enum Rotation, QuaternionT<T> &) {}
};

namespace AP {
// zero-initialised, without a constructor: in the Lua RAM .bss of the object that uses it
inline AP_CustomRotations &custom_rotations()
{
    static AP_CustomRotations custom_rotations;
    return custom_rotations;
}
}
