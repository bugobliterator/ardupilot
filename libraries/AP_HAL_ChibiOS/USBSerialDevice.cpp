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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(HAL_NO_UARTDRIVER) && HAL_USE_SERIAL_USB

#include "USBSerialDevice.h"
#include "GPIO.h"
#include <usbcfg.h>
#include "shared_dma.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

USBSerialDevice::USBSerialDevice(const UARTDriver::SerialDef &_sdef, ByteBuffer &readbuf, ByteBuffer &writebuf, HAL_Semaphore &write_mutex) :
sdef(_sdef),
_readbuf(readbuf),
_writebuf(writebuf),
_write_mutex(write_mutex)
{}

/*
* Initializes a serial-over-USB CDC driver.
*/
bool USBSerialDevice::open()
{
    if (sdef.serial == nullptr) {
        return false;
    }

    if (!_device_initialised) {
        if ((SerialUSBDriver*)sdef.serial == &SDU1
#if HAL_HAVE_DUAL_USB_CDC
            || (SerialUSBDriver*)sdef.serial == &SDU2
#endif
        ) {
            usb_initialise();
        }
        _device_initialised = true;
    }
    return true;
}


bool USBSerialDevice::close()
{
    sduStop((SerialUSBDriver*)sdef.serial);
    return true;
}

void USBSerialDevice::flush()
{
    sduSOFHookI((SerialUSBDriver*)sdef.serial);
}

/*
  write any pending bytes to the device
 */
void USBSerialDevice::write()
{
    // write any pending bytes
    uint32_t n = _writebuf.available();
    if (n <= 0) {
        return;
    }

    WITH_SEMAPHORE(_write_mutex);

    ByteBuffer::IoVec vec[2];
    uint16_t nwritten = 0;
    // this sets _in_write_call to true and back to false on function exit
    AutoFlagResetter<bool> write_call(_in_write_call, true, false);

    const auto n_vec = _writebuf.peekiovec(vec, n);
    for (int i = 0; i < n_vec; i++) {
        int ret = -1;
        ret = 0;
        ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);

        if (ret < 0) {
            break;
        }
        if (ret > 0) {
            nwritten += ret;
        }
        _writebuf.advance(ret);

        /* We wrote less than we asked for, stop */
        if ((unsigned)ret != vec[i].len) {
            break;
        }
    }

    _tx_stats_bytes += nwritten;
}

void USBSerialDevice::read()
{
    // don't try IO on a disconnected USB port
    if (((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE) {
        return;
    }

    AutoFlagResetter<bool> read_call(_in_read_call, true, false);
    ((GPIO *)hal.gpio)->set_usb_connected();

    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        int ret = 0;
        ret = chnReadTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
        if (ret < 0) {
            break;
        }
        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }

    if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
        chEvtSignal(_wait.thread_ctx, UARTDriver::EVT_DATA);
    }
}

bool USBSerialDevice::is_open() const
{
    return ((SerialUSBDriver*)sdef.serial)->config->usbp->state != USB_ACTIVE;
}


// record timestamp of new incoming data
void USBSerialDevice::receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}

// request information on uart I/O for @SYS/uarts.txt for this uart
void USBSerialDevice::get_info(ExpandingString &str)
{
    uint32_t now_ms = AP_HAL::millis();
    str.printf("OTG%u  ", unsigned(sdef.instance));

    str.printf("TX =%8u RX =%8u TXBD=%6u RXBD=%6u\n",
               unsigned(_tx_stats_bytes),
               unsigned(_rx_stats_bytes),
               unsigned(_tx_stats_bytes * 10000 / (now_ms - _last_stats_ms)),
               unsigned(_rx_stats_bytes * 10000 / (now_ms - _last_stats_ms)));
    _tx_stats_bytes = 0;
    _rx_stats_bytes = 0;
    _last_stats_ms = now_ms;
}

/*
  initialise the USB bus, called from both UARTDriver and stdio for startup debug
  This can be called before the hal is initialised so must not call any hal functions
 */
void usb_initialise(void)
{
    static bool initialised;
    if (initialised) {
        return;
    }
    initialised = true;
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg1);
#if HAL_HAVE_DUAL_USB_CDC
    sduObjectInit(&SDU2);
    sduStart(&SDU2, &serusbcfg2);
#endif
    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg1.usbp);
    chThdSleep(chTimeUS2I(1500));
    usbStart(serusbcfg1.usbp, &usbcfg);
    usbConnectBus(serusbcfg1.usbp);
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
