#pragma once
/*
  M4 build: the M7's custom rotations (ROTATION_CUSTOM_1/2) are parameters
  the M4 cannot read. Vector3::rotate and Quaternion::from_rotation (and so
  Quaternion::rotate) compile against these stubs, which leave the value
  unchanged. They are never reached: no local binding takes a Rotation
  (BindingLocal_generated.cpp), and the linker drops them from the image.
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
// zero-initialised, without a constructor: in the .bss of the object that uses it
inline AP_CustomRotations &custom_rotations()
{
    static AP_CustomRotations custom_rotations;
    return custom_rotations;
}
}
