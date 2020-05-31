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
 * Author: Siddharth Bharat Purohit
 * Referenced from implementation by Pavel Kirienko <pavel.kirienko@zubax.com>
 * for Zubax Babel
 */

#include "AP_SLCANIface.h"

#if HAL_NUM_CAN_IFACES
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include "AP_CANManager.h"

#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define LOG_TAG "SLCAN"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo SLCAN::CANIface::var_info[] = {
    // @Param: CPORT
    // @DisplayName: SLCAN Route
    // @Description: CAN Interface ID to be routed to SLCAN, 0 means no routing
    // @Values: 0:Disabled,1:First interface,2:Second interface
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CPORT", 1, SLCAN::CANIface, _slcan_can_port, 0),

    // @Param: SERNUM
    // @DisplayName: SLCAN Serial Port
    // @Description: Serial Port ID to be used for temporary SLCAN iface, -1 means no temporary serial. This parameter is automatically reset on reboot or on timeout. See CAN_SLCAN_TIMOUT for timeout details
    // @Values: -1:Disabled,0:Serial0,1:Serial1,2:Serial2,3:Serial3,4:Serial4,5:Serial5,6:Serial6
    // @User: Standard
    AP_GROUPINFO("SERNUM", 2, SLCAN::CANIface, _slcan_ser_port, -1),

    // @Param: TIMOUT
    // @DisplayName: SLCAN Timeout
    // @Description: Duration of inactivity after which SLCAN is switched back to original driver in seconds.
    // @Range: 0 32767
    // @User: Standard
    AP_GROUPINFO("TIMOUT", 3, SLCAN::CANIface, _slcan_timeout, 0),

    // @Param: MODE
    // @DisplayName: SLCAN MODE
    // @Description: SLCAN Mode to be selected, Passthrough mode routes all can drivers to and from SLCAN. Interface Mode puts selected driver on SLCAN instead of CAN bus, and Silent mode routes all packets on the selected CAN Bus
    // @Range: 0 2
    // @Values: 0: Passthrough Mode 1: Interface Mode 2: Silent Mode
    // @User: Standard
    AP_GROUPINFO("MODE", 4, SLCAN::CANIface, _slcan_mode, 0),

    AP_GROUPEND
};

////////Helper Methods//////////

static bool hex2nibble_error;

static uint8_t nibble2hex(uint8_t x)
{
    // Allocating in RAM because it's faster
    static uint8_t ConversionTable[] = {
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
    };
    return ConversionTable[x & 0x0F];
}

static uint8_t hex2nibble(char c)
{
    // Must go into RAM, not flash, because flash is slow
    static uint8_t NumConversionTable[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9
    };

    static uint8_t AlphaConversionTable[] = {
        10, 11, 12, 13, 14, 15
    };

    uint8_t out = 255;

    if (c >= '0' && c <= '9') {
        out = NumConversionTable[int(c) - int('0')];
    } else if (c >= 'a' && c <= 'f') {
        out = AlphaConversionTable[int(c) - int('a')];
    } else if (c >= 'A' && c <= 'F') {
        out = AlphaConversionTable[int(c) - int('A')];
    }

    if (out == 255) {
        hex2nibble_error = true;
    }
    return out;
}

int SLCAN::CANIface::set_port(AP_HAL::UARTDriver* port)
{
    if (port == nullptr) {
        return -1;
    }
    _port = port;
    return 0;
}


bool SLCAN::CANIface::push_Frame(AP_HAL::CANFrame &frame)
{
    AP_HAL::CANIface::CanRxItem frm;
    frm.frame = frame;
    frm.flags = 0;
    frm.timestamp_us = AP_HAL::native_micros64();
    return rx_queue_.push(frm);
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data>
 * The emitting functions below are highly optimized for speed.
 */
bool SLCAN::CANIface::handle_FrameDataExt(const char* cmd)
{
    AP_HAL::CANFrame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + AP_HAL::CANFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';
    if (f.dlc > AP_HAL::CANFrame::MaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[10];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANIface::handle_FrameDataStd(const char* cmd)
{
    AP_HAL::CANFrame f;
    hex2nibble_error = false;
    f.id = (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + AP_HAL::CANFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc > AP_HAL::CANFrame::MaxDataLen) {
        return false;
    }
    {
        const char* p = &cmd[5];
        for (unsigned i = 0; i < f.dlc; i++) {
            f.data[i] = (hex2nibble(*p) << 4) | hex2nibble(*(p + 1));
            p += 2;
        }
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANIface::handle_FrameRTRExt(const char* cmd)
{
    AP_HAL::CANFrame f;
    hex2nibble_error = false;
    f.id = f.FlagEFF | f.FlagRTR |
           (hex2nibble(cmd[1]) << 28) |
           (hex2nibble(cmd[2]) << 24) |
           (hex2nibble(cmd[3]) << 20) |
           (hex2nibble(cmd[4]) << 16) |
           (hex2nibble(cmd[5]) << 12) |
           (hex2nibble(cmd[6]) <<  8) |
           (hex2nibble(cmd[7]) <<  4) |
           (hex2nibble(cmd[8]) <<  0);
    if (cmd[9] < '0' || cmd[9] > ('0' + AP_HAL::CANFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[9] - '0';

    if (f.dlc > AP_HAL::CANFrame::MaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

bool SLCAN::CANIface::handle_FrameRTRStd(const char* cmd)
{
    AP_HAL::CANFrame f;
    hex2nibble_error = false;
    f.id = f.FlagRTR |
           (hex2nibble(cmd[1]) << 8) |
           (hex2nibble(cmd[2]) << 4) |
           (hex2nibble(cmd[3]) << 0);
    if (cmd[4] < '0' || cmd[4] > ('0' + AP_HAL::CANFrame::MaxDataLen)) {
        return false;
    }
    f.dlc = cmd[4] - '0';
    if (f.dlc <= AP_HAL::CANFrame::MaxDataLen) {
        return false;
    }
    if (hex2nibble_error) {
        return false;
    }
    return push_Frame(f);
}

static inline const char* getASCIIStatusCode(bool status)
{
    return status ? "\r" : "\a";
}

bool SLCAN::CANIface::is_initialized() const
{
    return initialized_;
}

bool SLCAN::CANIface::init_passthrough(uint8_t i)
{
    // We will not be running any driver on this port,
    // just forward everything we receive on this port
    if (initialized_ ||
        _slcan_can_port <= 0 ||
        _slcan_can_port != i+1) {
        return false;
    }

    _can_iface = hal.can[i];
    AP::can().log_text(AP_CANManager::LOG_INFO, LOG_TAG, "Launching SLCAN Passthrough thread for CAN%d\n", _slcan_can_port - 1);

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SLCAN::CANIface::slcan_passthrough_loop, void), "SLCAN_Passthrough", 1024, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        AP::can().log_text(AP_CANManager::LOG_ERROR, LOG_TAG, "Unable to launch slcan passthrough thread\n");
        return false;
    }
    return true;
}

/**
 * General frame format:
 *  <type> <id> <dlc> <data> [timestamp msec] [flags]
 * Types:
 *  R - RTR extended
 *  r - RTR standard
 *  T - Data extended
 *  t - Data standard
 * Flags:
 *  L - this frame is a loopback frame; timestamp field contains TX timestamp
 */
int16_t SLCAN::CANIface::reportFrame(const AP_HAL::CANFrame& frame, uint64_t timestamp_usec)
{
    if (_port == nullptr) {
        return -1;
    }
    constexpr unsigned SLCANMaxFrameSize = 40;
    uint8_t buffer[SLCANMaxFrameSize] = {'\0'};
    uint8_t* p = &buffer[0];
    /*
    * Frame type
    */
    if (frame.isRemoteTransmissionRequest()) {
        *p++ = frame.isExtended() ? 'R' : 'r';
    } else if (frame.isErrorFrame()) {
        return -1;     // Not supported
    } else {
        *p++ = frame.isExtended() ? 'T' : 't';
    }

    /*
    * ID
    */
    {
        const uint32_t id = frame.id & frame.MaskExtID;
        if (frame.isExtended()) {
            *p++ = nibble2hex(id >> 28);
            *p++ = nibble2hex(id >> 24);
            *p++ = nibble2hex(id >> 20);
            *p++ = nibble2hex(id >> 16);
            *p++ = nibble2hex(id >> 12);
        }
        *p++ = nibble2hex(id >> 8);
        *p++ = nibble2hex(id >> 4);
        *p++ = nibble2hex(id >> 0);
    }

    /*
    * DLC
    */
    *p++ = char('0' + frame.dlc);

    /*
    * Data
    */
    for (unsigned i = 0; i < frame.dlc; i++) {
        const uint8_t byte = frame.data[i];
        *p++ = nibble2hex(byte >> 4);
        *p++ = nibble2hex(byte);
    }

    /*
    * Timestamp
    */
    {
        // SLCAN format - [0, 60000) milliseconds
        const auto slcan_timestamp = uint16_t(timestamp_usec / 1000U);
        *p++ = nibble2hex(slcan_timestamp >> 12);
        *p++ = nibble2hex(slcan_timestamp >> 8);
        *p++ = nibble2hex(slcan_timestamp >> 4);
        *p++ = nibble2hex(slcan_timestamp >> 0);
    }

    /*
    * Finalization
    */
    *p++ = '\r';
    const auto frame_size = unsigned(p - &buffer[0]);

    if (_port->txspace() < _pending_frame_size) {
        _pending_frame_size = frame_size;
        return 0;
    }
    //Write to Serial
    if (!_port->write_locked(&buffer[0], frame_size, _serial_lock_key)) {
        return 0;
    }
    return 1;
}

//Accepts command string, returns response string or nullptr if no response is needed.
const char* SLCAN::CANIface::processCommand(char* cmd)
{

    if (_port == nullptr) {
        return nullptr;
    }

    /*
    * High-traffic SLCAN commands go first
    */
    if (cmd[0] == 'T') {
        return handle_FrameDataExt(cmd) ? "Z\r" : "\a";
    } else if (cmd[0] == 't') {
        return handle_FrameDataStd(cmd) ? "z\r" : "\a";
    } else if (cmd[0] == 'R') {
        return handle_FrameRTRExt(cmd) ? "Z\r" : "\a";
    } else if (cmd[0] == 'r' && cmd[1] <= '9') { // The second condition is needed to avoid greedy matching
        // See long commands below
        return handle_FrameRTRStd(cmd) ? "z\r" : "\a";
    }
    uint8_t resp_bytes[40];
    uint16_t resp_len;
    /*
    * Regular SLCAN commands
    */
    switch (cmd[0]) {
    case 'S':               // Set CAN bitrate
    case 'O':               // Open CAN in normal mode
    case 'L':               // Open CAN in listen-only mode
    case 'l':               // Open CAN with loopback enabled
    case 'C':               // Close CAN
    case 'M':               // Set CAN acceptance filter ID
    case 'm':               // Set CAN acceptance filter mask
    case 'U':               // Set UART baud rate, see http://www.can232.com/docs/can232_v3.pdf
    case 'Z': {             // Enable/disable RX and loopback timestamping
        return getASCIIStatusCode(true);    // Returning success for compatibility reasons
    }
    case 'F': {             // Get status flags
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes), "F%02X\r", unsigned(0));    // Returning success for compatibility reasons
        if (resp_len > 0) {
            _port->write_locked(resp_bytes, resp_len, _serial_lock_key);
        }
        return nullptr;
    }
    case 'V': {             // HW/SW version
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"V%x%x%x%x\r", 1, 0, 1, 0);
        if (resp_len > 0) {
            _port->write_locked(resp_bytes, resp_len, _serial_lock_key);
        }
        return nullptr;
    }
    case 'N': {             // Serial number
        const uint8_t uid_buf_len = 12;
        uint8_t uid_len = uid_buf_len;
        uint8_t unique_id[uid_buf_len];
        char buf[uid_buf_len * 2 + 1] = {'\0'};
        char* pos = &buf[0];
        if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
            for (uint8_t i = 0; i < uid_buf_len; i++) {
                *pos++ = nibble2hex(unique_id[i] >> 4);
                *pos++ = nibble2hex(unique_id[i]);
            }
        }
        *pos++ = '\0';
        resp_len = snprintf((char*)resp_bytes, sizeof(resp_bytes),"N%s\r", &buf[0]);
        if (resp_len > 0) {
            _port->write_locked(resp_bytes, resp_len, _serial_lock_key);
        }
        return nullptr;
    }
    default: {
        break;
    }
    }

    return getASCIIStatusCode(false);
}

// add bytes to parse the received SLCAN Data stream
inline void SLCAN::CANIface::addByte(const uint8_t byte)
{
    if (_port == nullptr) {
        return;
    }
    if ((byte >= 32 && byte <= 126)) {  // Normal printable ASCII character
        if (pos_ < SLCAN_BUFFER_SIZE) {
            buf_[pos_] = char(byte);
            pos_ += 1;
        } else {
            pos_ = 0;   // Buffer overrun; silently drop the data
        }
    } else if (byte == '\r') {  // End of command (SLCAN)

        // Processing the command
        buf_[pos_] = '\0';
        const char* const response = processCommand(reinterpret_cast<char*>(&buf_[0]));
        pos_ = 0;

        // Sending the response if provided
        if (response != nullptr) {

            _port->write_locked(reinterpret_cast<const uint8_t*>(response),
                                strlen(response), _serial_lock_key);
        }
    } else if (byte == 8 || byte == 127) {  // DEL or BS (backspace)
        if (pos_ > 0) {
            pos_ -= 1;
        }
    } else {    // This also includes Ctrl+C, Ctrl+D
        pos_ = 0;   // Invalid byte - drop the current command
    }
}

// send method to transmit the frame through SLCAN interface
int16_t SLCAN::CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, AP_HAL::CANIface::CanIOFlags flags)
{
    if (frame.isErrorFrame() || frame.dlc > 8) {
        return -1;
    }
    return reportFrame(frame, AP_HAL::native_micros64());
}

// receive method to read the frame recorded in the buffer
int16_t SLCAN::CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& rx_time,
                                 AP_HAL::CANIface::CanIOFlags& out_flags)
{
    if (_port == nullptr) {
        return 0;
    }
    if (!_port->available_locked(_serial_lock_key)) {
        return 0;
    } else {
        uint32_t num_bytes = _port->available_locked(_serial_lock_key);
        while (num_bytes--) {
            int16_t ret = _port->read_locked(_serial_lock_key);
            if (ret <= 0) {
                break;
            }
            addByte(ret);
        }
    }
    if (!rx_queue_.available()) {
        return 0;
    }
    CanRxItem frm;
    rx_queue_.pop(frm);
    out_frame = frm.frame;
    rx_time = frm.timestamp_us;
    out_flags = frm.flags;
    return 1;
}

// Thread loop method to create a passthough link between SLCAN and CAN Iface
void SLCAN::CANIface::slcan_passthrough_loop()
{
    uint8_t drv_num = _slcan_can_port - 1;
    AP_HAL::CANFrame frame;
    bool read_select;
    bool write_select;
    uint64_t timestamp_us;
    AP_HAL::CANIface::CanIOFlags flags;
    uint8_t prev_ser_port = 255;
    int8_t _slcan_ser_port_cache;
    bool had_activity;
    uint32_t last_had_activity = 0;
    while (true) {
        _slcan_ser_port_cache = _slcan_ser_port;
        while (_slcan_ser_port_cache < 0) {
            hal.scheduler->delay(100);
            _slcan_ser_port_cache = _slcan_ser_port;
        }

        if (prev_ser_port != _slcan_ser_port_cache) {
            AP_HAL::UARTDriver* slcan_serial = AP::serialmanager().get_serial_by_id(_slcan_ser_port_cache);
            if (slcan_serial != _port) {
                _port = slcan_serial;
            } else {
                _slcan_ser_port.set_and_save(-1);
                _slcan_ser_port_cache = -1;
                continue;
            }
            _port->lock_port(_serial_lock_key, _serial_lock_key);
            prev_ser_port = _slcan_ser_port_cache;
            gcs().send_text(MAV_SEVERITY_INFO, "CANManager: Starting SLCAN Passthrough on Serial %d with CAN%d", _slcan_ser_port_cache, drv_num);
            last_had_activity = AP_HAL::native_millis();
        }
        read_select = true;
        write_select = true;
        _can_iface->select(read_select, write_select, nullptr, 0);
        if (read_select) {
            //read data from can and put on slcan
            if (_can_iface->receive(frame, timestamp_us, flags) > 0) {
                send(frame, AP_HAL::native_micros() + 1000, flags);
            }
        }
        //read data from slcan and put on can
        if (receive(frame, timestamp_us, flags) > 0) {
            _can_iface->send(frame, AP_HAL::native_micros() + 1000, flags);
            had_activity  = true;
        }

        if (had_activity) {
            last_had_activity = AP_HAL::native_millis();
        }
        if (((AP_HAL::native_millis() - last_had_activity) > (uint32_t)_slcan_timeout) &&
            (uint32_t)_slcan_timeout != 0) {
            _port->lock_port(0, 0);
            _slcan_ser_port.set_and_save(-1);
        }
        hal.scheduler->delay_microseconds(100);
    }
}

void SLCAN::CANIface::reset_params()
{
    _slcan_ser_port.set_and_save(-1);
}
#endif
