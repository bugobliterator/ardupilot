#pragma once
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(HAL_NO_UARTDRIVER)
#include <stdint.h>
#include <stdlib.h>
#include "SerialDevice.h"

class ChibiOS::UARTDevice : public ChibiOS::SerialDevice {
public:
    UARTDevice(const ChibiOS::UARTDriver::SerialDef &sdef, ByteBuffer &readbuf, ByteBuffer &writebuf, HAL_Semaphore &write_mutex, uint32_t baudrate);
    virtual ~UARTDevice() {}

    bool open() override;
    bool close() override;
    void write() override;
    void read() override;
    void flush() override;
    AP_HAL::UARTDriver::flow_control get_flow_control(void) override { return _flow_control; }
    void set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting) override;
    bool is_dma_enabled() const override { return rx_dma_enabled && tx_dma_enabled; }
    
    bool set_options(uint16_t options) override;
    uint16_t get_options() const override;

    bool tx_busy() override { return _in_write_call; }
    bool rx_busy() override { return _in_read_call; }

    void get_info(ExpandingString &str) override;
    bool is_open() const override { return true; }
    void configure_parity(uint8_t v) override;
    void set_stop_bits(uint8_t n) override;
    uint64_t get_rx_timestamp_us() override { return _receive_timestamp[_receive_timestamp_idx]; }
    /*
      software control of the CTS/RTS pins if available. Return false if
      not available
     */
    bool set_RTS_pin(bool high) override;
    bool set_CTS_pin(bool high) override;

    // update rts state
    bool get_rts_active() override { return _rts_is_active; }
    void update_rts_line() override;

    // set thread ctx
    void set_uart_thread_ctx(thread_t* _uart_thread_ctx) override { uart_thread_ctx = _uart_thread_ctx; }

private:
    const ChibiOS::UARTDriver::SerialDef &sdef;
    uint32_t _baudrate;
    bool rx_dma_enabled;
    bool tx_dma_enabled;

    /*
      copy of rx_line, tx_line, rts_line and cts_line with alternative configs resolved
     */
    ioline_t atx_line;
    ioline_t arx_line;
    ioline_t arts_line;
    ioline_t acts_line;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
#ifndef HAL_UART_NODMA
    volatile uint8_t rx_bounce_idx;
    uint8_t *rx_bounce_buf[2];
    uint8_t *tx_bounce_buf;
    uint16_t contention_counter;
#endif

#ifndef HAL_UART_NODMA
    const stm32_dma_stream_t* rxdma;
    const stm32_dma_stream_t* txdma;
#endif

#ifndef HAL_UART_NODMA
    Shared_DMA *dma_handle;
#endif

    // timestamp for receiving data on the UART, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;

    // we remember config options from set_options to apply on sdStart()
    uint32_t _cr1_options;
    uint32_t _cr2_options;
    uint32_t _cr3_options;
    uint16_t _last_options;

    // half duplex control. After writing we throw away bytes for 4 byte widths to
    // prevent reading our own bytes back
#if CH_CFG_USE_EVENTS == TRUE
    bool half_duplex;
    event_listener_t hd_listener;
    eventflags_t hd_tx_active;
    void half_duplex_setup_tx(void);
#endif

#if CH_CFG_USE_EVENTS == TRUE
    // listener for parity error events
    event_listener_t ev_listener;
    bool parity_enabled;
#endif

#if HAL_USE_SERIAL == TRUE
    SerialConfig sercfg;
#endif

    volatile bool _in_write_call;
    volatile bool _in_read_call;
    HAL_Semaphore &_write_mutex;
    volatile bool _device_initialised;
    ByteBuffer &_readbuf;
    ByteBuffer &_writebuf;

    // statistics
    uint32_t _tx_stats_bytes;
    uint32_t _rx_stats_bytes;
    uint32_t _last_stats_ms;
    uint32_t _last_write_call_us;

    // handling of flow control
    AP_HAL::UARTDriver::flow_control _flow_control = AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE;
    bool _rts_is_active;
    uint32_t _last_write_completed_us;
    uint32_t _first_write_started_us;
    uint32_t _total_written;

    // thread used for writing and reading
    thread_t* volatile uart_thread_ctx;

#ifndef HAL_UART_NODMA
    static void rx_irq_cb(void* sd);
#endif
    static void rxbuff_full_irq(void* self, uint32_t flags);
    static void tx_complete(void* self, uint32_t flags);

#ifndef HAL_UART_NODMA
    void dma_tx_allocate(Shared_DMA *ctx);
    void dma_tx_deallocate(Shared_DMA *ctx);
    void dma_rx_enable(void);
#endif

    void check_dma_tx_completion(void);
#ifndef HAL_UART_NODMA
    void write_pending_bytes_DMA(uint32_t n);
#endif
    void write_pending_bytes_NODMA(uint32_t n);
    void write_pending_bytes(void);
    void read_bytes_NODMA();

    void receive_timestamp_update(void);

    // set SERIALn_OPTIONS for pullup/pulldown
    void set_pushpull(uint16_t options);
};
#endif // CONFIG_HAL_BOARD
