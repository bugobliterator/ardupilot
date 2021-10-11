/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_ChibiOS.h"
#include "shared_dma.h"
#include "Semaphores.h"

#define RX_BOUNCE_BUFSIZE 64U
#define TX_BOUNCE_BUFSIZE 64U

// enough for uartA to uartI, plus IOMCU
#define UART_MAX_DRIVERS 10

class ChibiOS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uint8_t serial_num);

    /* Do not allow copies */
    UARTDriver(const UARTDriver &other) = delete;
    UARTDriver &operator=(const UARTDriver&) = delete;

    void begin(uint32_t b) override;
    void begin_locked(uint32_t b, uint32_t write_key) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;
    uint32_t get_usb_baud() const override;

    uint32_t available() override;
    uint32_t available_locked(uint32_t key) override;

    uint32_t txspace() override;
    int16_t read() override;
    ssize_t read(uint8_t *buffer, uint16_t count) override;
    int16_t read_locked(uint32_t key) override;
    void _rx_timer_tick(void);
    void _tx_timer_tick(void);

    bool discard_input() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    // lock a port for exclusive use. Use a key of 0 to unlock
    bool lock_port(uint32_t write_key, uint32_t read_key) override;

    // control optional features
    bool set_options(uint16_t options) override;
    uint16_t get_options(void) const override;

    // write to a locked port. If port is locked and key is not correct then 0 is returned
    // and write is discarded
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key) override;

    struct SerialDef {
        BaseSequentialStream* serial;
        uint8_t instance;
        bool is_usb;
#ifndef HAL_UART_NODMA
        bool dma_rx;
        uint8_t dma_rx_stream_id;
        uint32_t dma_rx_channel_id;
        bool dma_tx;
        uint8_t dma_tx_stream_id;
        uint32_t dma_tx_channel_id;
#endif
        ioline_t tx_line;
        ioline_t rx_line;
        ioline_t rts_line;
        ioline_t cts_line;
        int8_t rxinv_gpio;
        uint8_t rxinv_polarity;
        int8_t txinv_gpio;
        uint8_t txinv_polarity;
        uint8_t endpoint_id;
        uint8_t get_index(void) const {
            return uint8_t(this - &_serial_tab[0]);
        }
    };

    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override;

    void set_flow_control(enum flow_control flow_control) override;
    enum flow_control get_flow_control(void) override;

    // allow for low latency writes
    bool set_unbuffered_writes(bool on) override;

    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;

    /*
      software control of the CTS/RTS pins if available. Return false if
      not available
     */
    bool set_RTS_pin(bool high) override;
    bool set_CTS_pin(bool high) override;

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.

      A return value of zero means the HAL does not support this API
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    uint32_t bw_in_kilobytes_per_second() const override {
        if (sdef.is_usb) {
            return 200;
        }
        return _baudrate/(9*1024);
    }

    // request information on uart I/O for one uart
    void uart_info(ExpandingString &str) override;

    /*
      return true if this UART has DMA enabled on both RX and TX
     */
    bool is_dma_enabled() const override;

    // event used to wake up waiting thread. This event number is for
    // caller threads
    static const eventmask_t EVT_DATA = EVENT_MASK(10);

    // event for parity error
    static const eventmask_t EVT_PARITY = EVENT_MASK(11);

    // event for transmit end for half-duplex
    static const eventmask_t EVT_TRANSMIT_END = EVENT_MASK(12);

    // events for dma tx, thread per UART so can be from 0
    static const eventmask_t EVT_TRANSMIT_DMA_START = EVENT_MASK(0);
    static const eventmask_t EVT_TRANSMIT_DMA_COMPLETE = EVENT_MASK(1);
    static const eventmask_t EVT_TRANSMIT_DATA_READY = EVENT_MASK(2);
    static const eventmask_t EVT_TRANSMIT_UNBUFFERED = EVENT_MASK(3);

private:
    const SerialDef &sdef;
    AP_HAL::OwnPtr<ChibiOS::SerialDevice> _device;

    // thread used for all UARTs
    static thread_t* volatile uart_rx_thread_ctx;

    // table to find UARTDrivers from serial number, used for event handling
    static UARTDriver *uart_drivers[UART_MAX_DRIVERS];
    
    // thread used for writing and reading
    thread_t* volatile uart_thread_ctx;
    char uart_thread_name[6];

    // index into uart_drivers table
    uint8_t serial_num;

    // key for a locked port
    uint32_t lock_write_key;
    uint32_t lock_read_key;

    uint32_t _baudrate;

    const thread_t* _uart_owner_thd;

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    HAL_Semaphore _write_mutex;

    volatile bool _in_rx_timer;
    bool _blocking_writes;
    volatile bool _rx_initialised;
    volatile bool _tx_initialised;

    static const SerialDef _serial_tab[];

    // set to true for unbuffered writes (low latency writes)
    bool unbuffered_writes;


    static void thread_rx_init();
    void thread_init();
    void uart_thread();
    static void uart_rx_thread(void* arg);
    static void uart_thread_trampoline(void* p);
};

// access to usb init for stdio.cpp
void usb_initialise(void);
