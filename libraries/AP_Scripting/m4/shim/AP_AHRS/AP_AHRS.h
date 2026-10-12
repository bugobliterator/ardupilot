#pragma once
// M4 Lua build: the Location methods that read home or the EKF origin stay remote, so
// Location.cpp is built without them
#define AP_AHRS_ENABLED 0
