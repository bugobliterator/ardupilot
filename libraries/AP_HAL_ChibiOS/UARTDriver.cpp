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
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(HAL_NO_UARTDRIVER)
#include "UARTDriver.h"
#include "GPIO.h"
#include <usbcfg.h>
#include "shared_dma.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "USBSerialDevice.h"
#include "UARTDevice.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

#if HAL_WITH_IO_MCU
extern ChibiOS::UARTDriver uart_io;
#endif

const UARTDriver::SerialDef UARTDriver::_serial_tab[] = { HAL_UART_DEVICE_LIST };

// handle for UART handling thread
thread_t* volatile UARTDriver::uart_rx_thread_ctx;

// table to find UARTDrivers from serial number, used for event handling
UARTDriver *UARTDriver::uart_drivers[UART_MAX_DRIVERS];

#ifndef HAL_UART_MIN_TX_SIZE
#define HAL_UART_MIN_TX_SIZE 512
#endif

#ifndef HAL_UART_MIN_RX_SIZE
#define HAL_UART_MIN_RX_SIZE 512
#endif

#ifndef HAL_UART_STACK_SIZE
#define HAL_UART_STACK_SIZE 320
#endif

#ifndef HAL_UART_RX_STACK_SIZE
#define HAL_UART_RX_STACK_SIZE 768
#endif

UARTDriver::UARTDriver(uint8_t _serial_num) :
serial_num(_serial_num),
sdef(_serial_tab[_serial_num]),
_baudrate(57600)
{
    osalDbgAssert(serial_num < UART_MAX_DRIVERS, "too many UART drivers");
    uart_drivers[serial_num] = this;
}

/*
  thread for handling UART send/receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDriver::uart_thread()
{
    while (uart_thread_ctx == nullptr) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        eventmask_t mask = chEvtWaitAnyTimeout(EVT_TRANSMIT_DATA_READY | EVT_TRANSMIT_END | EVT_TRANSMIT_UNBUFFERED, chTimeMS2I(1));

        // change the thread priority if requested - if unbuffered it should only have higher priority than the owner so that
        // handoff occurs immediately
        if (mask & EVT_TRANSMIT_UNBUFFERED) {
            chThdSetPriority(unbuffered_writes ? MIN(_uart_owner_thd->realprio + 1, APM_UART_UNBUFFERED_PRIORITY) : APM_UART_PRIORITY);
        }
        // send more data
        if (_tx_initialised) {
            // now do the write
            _device->write();
        }
    }
}
#pragma GCC diagnostic pop

/*
  thread for handling UART receive

  We use events indexed by serial_num to trigger a more rapid send for
  unbuffered_write uarts, and run at 1kHz for general UART handling
 */
void UARTDriver::uart_rx_thread(void* arg)
{
    while (uart_rx_thread_ctx == nullptr) {
        hal.scheduler->delay_microseconds(1000);
    }

    while (true) {
        hal.scheduler->delay_microseconds(1000);

        for (uint8_t i=0; i<UART_MAX_DRIVERS; i++) {
            if (uart_drivers[i] == nullptr) {
                continue;
            }
            if (uart_drivers[i]->_rx_initialised) {
                uart_drivers[i]->_device->read();
            }
        }
    }
}

/*
  initialise UART RX thread
 */
void UARTDriver::thread_rx_init(void)
{
    if (uart_rx_thread_ctx == nullptr) {
        uart_rx_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(HAL_UART_RX_STACK_SIZE),
                                              "SERIAL_RX",
                                              APM_UART_PRIORITY,
                                              uart_rx_thread,
                                              nullptr);
        if (uart_rx_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART RX thread\n");
        }
    }
}

/*
  initialise UART TX_thread
 */
void UARTDriver::thread_init(void)
{
    if (uart_thread_ctx == nullptr && _device) {
        hal.util->snprintf(uart_thread_name, sizeof(uart_thread_name), sdef.is_usb ? "OTG%1u" : "UART%1u", sdef.instance);
        uart_thread_ctx = thread_create_alloc(THD_WORKING_AREA_SIZE(HAL_UART_STACK_SIZE),
                                              uart_thread_name,
                                              unbuffered_writes ? APM_UART_UNBUFFERED_PRIORITY : APM_UART_PRIORITY,
                                              uart_thread_trampoline,
                                              this);
        if (uart_thread_ctx == nullptr) {
            AP_HAL::panic("Could not create UART TX thread\n");
        }
        _device->set_uart_thread_ctx(uart_thread_ctx);
    }
}

void UARTDriver::uart_thread_trampoline(void* p)
{
    UARTDriver* uart = static_cast<UARTDriver*>(p);
    uart->uart_thread();
}

#ifndef HAL_STDOUT_SERIAL
/*
  hook to allow printf() to work on hal.console when we don't have a
  dedicated debug console
 */
static int hal_console_vprintf(const char *fmt, va_list arg)
{
    hal.console->vprintf(fmt, arg);
    return 1; // wrong length, but doesn't matter for what this is used for
}
#endif

void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS)
{
    thread_rx_init();

    if (sdef.serial == nullptr) {
        return;
    }
    uint16_t min_tx_buffer = HAL_UART_MIN_TX_SIZE;
    uint16_t min_rx_buffer = HAL_UART_MIN_RX_SIZE;

    /*
      increase min RX size to ensure we can receive a fully utilised
      UART if we are running our receive loop at 40Hz. This means 25ms
      of data. Assumes 10 bits per byte, which is normal for most
      protocols
     */
    bool rx_size_by_baudrate = true;
#if HAL_WITH_IO_MCU
    if (this == &uart_io) {
        // iomcu doesn't need extra space, just speed
        rx_size_by_baudrate = false;
        min_tx_buffer = 0;
        min_rx_buffer = 0;
    }
#endif
    if (rx_size_by_baudrate) {
        min_rx_buffer = MAX(min_rx_buffer, b/(40*10));
    }

    if (sdef.is_usb) {
        // give more buffer space for log download on USB
        min_tx_buffer *= 2;
    }

#if HAL_MEM_CLASS >= HAL_MEM_CLASS_500
    // on boards with plenty of memory we can use larger buffers
    min_tx_buffer *= 2;
    min_rx_buffer *= 2;
#endif

    // on PX4 we have enough memory to have a larger transmit and
    // receive buffer for all ports. This means we don't get delays
    // while waiting to write GPS config packets
    if (txS < min_tx_buffer) {
        txS = min_tx_buffer;
    }
    if (rxS < min_rx_buffer) {
        rxS = min_rx_buffer;
    }

    if (!_device) {
#if HAL_USE_SERIAL_USB
        if (sdef.is_usb) {
            _device = new USBSerialDevice(sdef, _readbuf, _writebuf, _write_mutex);
        } else 
#endif
#if HAL_USE_SERIAL
        {
            _device = new UARTDevice(sdef, _readbuf, _writebuf, _write_mutex, _baudrate);
        }
#endif
        if (!_device) {
            AP_BoardConfig::allocation_error("Failed to allocate SerialDevice");
        }
    }

    /*
      allocate the read buffer
      we allocate buffers before we successfully open the device as we
      want to allocate in the early stages of boot, and cause minimum
      thrashing of the heap once we are up. The ttyACM0 driver may not
      connect for some time after boot
     */
    while (_device->rx_busy()) {
        hal.scheduler->delay(1);
    }
    if (rxS != _readbuf.get_size()) {
        _rx_initialised = false;
        _readbuf.set_size(rxS);
    }

    bool clear_buffers = false;
    if (b != 0) {
        // clear buffers on baudrate change, but not on the console (which is usually USB)
        if (_baudrate != b && hal.console != this) {
            clear_buffers = true;
        }
        _baudrate = b;
    }

    if (clear_buffers) {
        _readbuf.clear();
    }

    /*
      allocate the write buffer
     */
    while (_device->tx_busy()) {
        hal.scheduler->delay(1);
    }
    if (txS != _writebuf.get_size()) {
        _tx_initialised = false;
        _writebuf.set_size(txS);
    }

    if (clear_buffers) {
        _writebuf.clear();
    }
    
    // Open SerialDevice for communication
    _device->open();
    
    if (_writebuf.get_size()) {
        _tx_initialised = true;
    }
    if (_readbuf.get_size()) {
        _rx_initialised = true;
    }
    _uart_owner_thd = chThdGetSelfX();
    // initialize the TX thread if necessary
    thread_init();

    if (serial_num == 0 && _tx_initialised) {
#ifndef HAL_STDOUT_SERIAL
        // setup hal.console to take printf() output
        vprintf_console_hook = hal_console_vprintf;
#endif
    }
}

void UARTDriver::begin(uint32_t b)
{
    if (lock_write_key != 0) {
        return;
    }
    begin(b, 0, 0);
}

void UARTDriver::begin_locked(uint32_t b, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        return;
    }
    begin(b, 0, 0);
}

void UARTDriver::end()
{
    if (!_device) {
        return;
    }
    while (_device->rx_busy()) hal.scheduler->delay(1);
    _rx_initialised = false;
    while (_device->tx_busy()) hal.scheduler->delay(1);
    _tx_initialised = false;

    _device->close();
    _readbuf.set_size(0);
    _writebuf.set_size(0);
}

void UARTDriver::flush()
{
    if (!_device) {
        return;
    }
    _device->flush();
}

bool UARTDriver::is_initialized()
{
    return _tx_initialised && _rx_initialised;
}

void UARTDriver::set_blocking_writes(bool blocking)
{
    _blocking_writes = blocking;
}

bool UARTDriver::tx_pending() { return _writebuf.available() > 0; }


/*
    get the requested usb baudrate - 0 = none
*/
uint32_t UARTDriver::get_usb_baud() const
{
#if HAL_USE_SERIAL_USB
    if (sdef.is_usb) {
        return ::get_usb_baud(sdef.endpoint_id);
    }
#endif
    return 0;
}

/* Empty implementations of Stream virtual methods */
uint32_t UARTDriver::available() {
    if (!_rx_initialised || lock_read_key) {
        return 0;
    }
    if (!_device || !_device->is_open()) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::available_locked(uint32_t key)
{
    if (lock_read_key != 0 && key != lock_read_key) {
        return -1;
    }
    if (!_device || !_device->is_open()) {
        return 0;
    }
    return _readbuf.available();
}

uint32_t UARTDriver::txspace()
{
    if (!_tx_initialised) {
        return 0;
    }
    return _writebuf.space();
}

bool UARTDriver::discard_input()
{
    if (!_device) {
        return -1;
    }
    if (lock_read_key != 0 || _uart_owner_thd != chThdGetSelfX()){
        return false;
    }
    if (!_rx_initialised) {
        return false;
    }

    _readbuf.clear();

    if (!_device->get_rts_active()) {
        _device->update_rts_line();
    }

    return true;
}

ssize_t UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    if (!_device) {
        return -1;
    }
    if (lock_read_key != 0 || _uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }

    const uint32_t ret = _readbuf.read(buffer, count);
    if (ret == 0) {
        return 0;
    }

    if (!_device->get_rts_active()) {
        _device->update_rts_line();
    }

    return ret;
}

int16_t UARTDriver::read()
{
    if (!_device) {
        return -1;
    }
    if (lock_read_key != 0 || _uart_owner_thd != chThdGetSelfX()){
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    if (!_device->get_rts_active()) {
        _device->update_rts_line();
    }

    return byte;
}

int16_t UARTDriver::read_locked(uint32_t key)
{
    if (!_device) {
        return -1;
    }
    if (lock_read_key != 0 && key != lock_read_key) {
        return -1;
    }
    if (!_rx_initialised) {
        return -1;
    }
    uint8_t byte;
    if (!_readbuf.read_byte(&byte)) {
        return -1;
    }
    if (!_device->get_rts_active()) {
        _device->update_rts_line();
    }
    return byte;
}

/* write one byte to the port */
size_t UARTDriver::write(uint8_t c)
{
    if (lock_write_key != 0) {
        return 0;
    }
    _write_mutex.take_blocking();

    if (!_tx_initialised) {
        _write_mutex.give();
        return 0;
    }

    while (_writebuf.space() == 0) {
        if (!_blocking_writes || unbuffered_writes) {
            _write_mutex.give();
            return 0;
        }
        // release the semaphore while sleeping
        _write_mutex.give();
        hal.scheduler->delay(1);
        _write_mutex.take_blocking();
    }
    size_t ret = _writebuf.write(&c, 1);
    if (unbuffered_writes) {
        chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_DATA_READY);
    }
    _write_mutex.give();
    return ret;
}

/* write a block of bytes to the port */
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (!_tx_initialised || lock_write_key != 0) {
		return 0;
	}

    if (_blocking_writes && !unbuffered_writes) {
        /*
          use the per-byte delay loop in write() above for blocking writes
         */
        size_t ret = 0;
        while (size--) {
            if (write(*buffer++) != 1) break;
            ret++;
        }
        return ret;
    }

    WITH_SEMAPHORE(_write_mutex);

    size_t ret = _writebuf.write(buffer, size);
    if (unbuffered_writes) {
        chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_DATA_READY);
    }
    return ret;
}

/*
  lock the uart for exclusive use by write_locked() and read_locked() with the right key
 */
bool UARTDriver::lock_port(uint32_t write_key, uint32_t read_key)
{
    if (lock_write_key && write_key != lock_write_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    if (lock_read_key && read_key != lock_read_key && read_key != 0) {
        // someone else is using it
        return false;
    }
    lock_write_key = write_key;
    lock_read_key = read_key;
    return true;
}

/*
   write to a locked port. If port is locked and key is not correct then 0 is returned
   and write is discarded. All writes are non-blocking
*/
size_t UARTDriver::write_locked(const uint8_t *buffer, size_t size, uint32_t key)
{
    if (lock_write_key != 0 && key != lock_write_key) {
        return 0;
    }
    WITH_SEMAPHORE(_write_mutex);
    return _writebuf.write(buffer, size);
}

/*
  wait for data to arrive, or a timeout. Return true if data has
  arrived, false on timeout
 */
bool UARTDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    uint32_t t0 = AP_HAL::millis();
    while (available() < n) {
        chEvtGetAndClearEvents(EVT_DATA);
        _device->set_rx_bytes_wait(chThdGetSelfX(), n);
        uint32_t now = AP_HAL::millis();
        if (now - t0 >= timeout_ms) {
            break;
        }
        chEvtWaitAnyTimeout(EVT_DATA, chTimeMS2I(timeout_ms - (now - t0)));
    }
    return available() >= n;
}

/*
  change flow control mode for port
 */
void UARTDriver::set_flow_control(enum flow_control flowcontrol)
{
    if (!is_initialized()) {
        // not ready yet, we just set variable for when we call begin
        return;
    }
    if (!_device) {
        return;
    }
    _device->set_flow_control(flowcontrol);
}

// get current flow control config
enum AP_HAL::UARTDriver::flow_control UARTDriver::get_flow_control()
{
    if (!_device) {
        return FLOW_CONTROL_DISABLE;
    }    
    return _device->get_flow_control();
}

/*
   setup unbuffered writes for lower latency
 */
bool UARTDriver::set_unbuffered_writes(bool on)
{
    unbuffered_writes = on;
    chEvtSignal(uart_thread_ctx, EVT_TRANSMIT_UNBUFFERED);
    return true;
}

/*
  setup parity
 */
void UARTDriver::configure_parity(uint8_t v)
{
    if (!_device) {
        return;
    }
    _device->configure_parity(v);
}

/*
  set stop bits
 */
void UARTDriver::set_stop_bits(int n)
{
    if (!_device) {
        return;
    }
    _device->set_stop_bits(n);
}

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
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _device->get_rx_timestamp_us();
    if (_baudrate > 0 && !sdef.is_usb) {
        // assume 10 bits per byte. For USB we assume zero transport delay
        uint32_t transport_time_us = (1000000UL * 10UL / _baudrate) * (nbytes + available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

// set optional features, return true on success
bool UARTDriver::set_options(uint16_t options)
{
    if (!_device) {
        return false;
    }
    return _device->set_options(options);
}

// get optional features
uint16_t UARTDriver::get_options(void) const
{
    if (!_device) {
        return 0;
    }
    return _device->get_options();
}

// request information on uart I/O for @SYS/uarts.txt for this uart
void UARTDriver::uart_info(ExpandingString &str)
{
    if (!_device) {
        return;
    }
    return _device->get_info(str);
}

bool UARTDriver::is_dma_enabled() const
{
    if (!_device) {
        return false;
    }

    return _device->is_dma_enabled();
}

bool UARTDriver::set_RTS_pin(bool high)
{
    if (!_device) {
        return false;
    }

    return _device->set_RTS_pin(high);
}

bool UARTDriver::set_CTS_pin(bool high)
{
    if (!_device) {
        return false;
    }

    return _device->set_CTS_pin(high);
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
