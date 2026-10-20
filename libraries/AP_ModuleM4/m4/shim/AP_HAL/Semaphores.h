#pragma once
// M4 build: BindingSupport.h names AP_HAL::Semaphore; the M4 image never constructs one
#include <stdint.h>
// as the real header: BindingSupport.h takes CLASS_NO_COPY from it
#include <AP_Common/AP_Common.h>
namespace AP_HAL {
class Semaphore {
public:
    virtual ~Semaphore() {}
    virtual bool take(uint32_t timeout_ms) = 0;
    virtual bool take_nonblocking() = 0;
    virtual void take_blocking() = 0;
    virtual bool give() = 0;
};
}
