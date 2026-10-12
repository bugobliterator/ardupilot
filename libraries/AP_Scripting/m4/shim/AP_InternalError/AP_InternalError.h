#pragma once
// M4 Lua build: a script on the M4 cannot latch the vehicle's internal-error flag;
// the argument is not evaluated
#define AP_INTERNALERROR_ENABLED 0
#define INTERNAL_ERROR(error_number) do { } while (0)
