#pragma once
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(HAL_NO_UARTDRIVER) && defined(HAL_USB_VENDOR_ID)
#include <stdint.h>
#include <stdlib.h>
#include "SerialDevice.h"

class ChibiOS::USBSerialDevice : public ChibiOS::SerialDevice {
public:
    USBSerialDevice(const ChibiOS::UARTDriver::SerialDef &sdef, ByteBuffer &readbuf, ByteBuffer &writebuf, HAL_Semaphore &write_mutex);
    virtual ~USBSerialDevice() {}
    bool open() override;
    bool close() override;
    void write() override;
    void read() override;
    void flush() override;

    bool tx_busy() override { return _in_write_call; }
    bool rx_busy() override { return _in_read_call; }

    void get_info(ExpandingString &str) override;
    bool is_open() const override;
    uint64_t get_rx_timestamp_us() override { return _receive_timestamp[_receive_timestamp_idx]; }

private:

    const ChibiOS::UARTDriver::SerialDef &sdef;
    ByteBuffer &_readbuf;
    ByteBuffer &_writebuf;

    // timestamp for receiving data on the UART, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;

    volatile bool _in_write_call;
    volatile bool _in_read_call;
    HAL_Semaphore &_write_mutex;

    // statistics
    uint32_t _tx_stats_bytes;
    uint32_t _rx_stats_bytes;
    uint32_t _last_stats_ms;

    volatile bool _device_initialised;

    void receive_timestamp_update();

};
#endif // CONFIG_HAL_BOARD
