#pragma once
// M4 build: AP_Math.h declares an AP_Vector3f parameter type and names AP_Float in
// static_asserts; the M4 has no parameters
#define AP_PARAMDEFV(_t, _suffix, _pt)
class AP_Float {};
