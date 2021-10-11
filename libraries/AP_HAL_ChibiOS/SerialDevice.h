#pragma once

#include <AP_HAL/AP_HAL.h>
#include <stdint.h>
#include <stdlib.h>
#include "UARTDriver.h"

class ChibiOS::SerialDevice {
public:
    virtual ~SerialDevice() {}

    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual void write() = 0;
    virtual void read() = 0;
    virtual AP_HAL::UARTDriver::flow_control get_flow_control(void) { return AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE; }
    virtual void set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting) {}
    virtual bool is_dma_enabled() const { return false; }
    // control optional features
    virtual bool set_options(uint16_t options) { return options==0; }
    virtual uint16_t get_options(void) const { return 0; }

    // flush any buffered data
    virtual void flush() {}

    virtual bool tx_busy() { return true; }
    virtual bool rx_busy() { return true; }

    virtual void get_info(ExpandingString &str) {}
    virtual bool is_open() const = 0; 
    virtual void configure_parity(uint8_t v) {}
    virtual void set_stop_bits(uint8_t n) {}
    virtual uint64_t get_rx_timestamp_us() { return 0; }
    /*
      software control of the CTS/RTS pins if available. Return false if
      not available
     */
    virtual bool set_RTS_pin(bool high) { return false; }
    virtual bool set_CTS_pin(bool high) { return false; }

    // update rts state
    virtual bool get_rts_active() { return true; }
    virtual void update_rts_line() {}

    void set_rx_bytes_wait(thread_t *thread_ctx, uint16_t n) {
        _wait.n = n;
        _wait.thread_ctx = thread_ctx;
    }

    virtual void set_uart_thread_ctx(thread_t* _uart_thread_ctx) {}

protected:
    struct {
        // thread waiting for data
        thread_t *thread_ctx;
        // number of bytes needed
        uint16_t n;
    } _wait;
};
