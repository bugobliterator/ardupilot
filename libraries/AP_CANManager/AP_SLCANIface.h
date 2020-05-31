
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_NUM_CAN_IFACES
#include "AP_HAL/utility/RingBuffer.h"
#include <AP_Param/AP_Param.h>

#define SLCAN_BUFFER_SIZE 200
#define SLCAN_RX_QUEUE_SIZE 64

#ifndef HAL_CAN_RX_STORAGE_SIZE
#define HAL_CAN_RX_QUEUE_SIZE 128
#endif

static_assert(HAL_CAN_RX_QUEUE_SIZE <= 254, "Invalid CAN Rx queue size");

namespace SLCAN
{

class CANIface: public AP_HAL::CANIface
{

    int16_t reportFrame(const AP_HAL::CANFrame& frame, uint64_t timestamp_usec);

    const char* processCommand(char* cmd);

    // pushes received frame into queue, false if failed
    bool push_Frame(AP_HAL::CANFrame &frame);

    // Methods to handle different types of frames,
    // return true if successfully received frame type
    bool handle_FrameRTRStd(const char* cmd);
    bool handle_FrameRTRExt(const char* cmd);
    bool handle_FrameDataStd(const char* cmd);
    bool handle_FrameDataExt(const char* cmd);

    // main passthrough loop running on a thread
    void slcan_passthrough_loop();

    // Parsing bytes received on the serial port
    inline void addByte(const uint8_t byte);

    bool initialized_;

    char buf_[SLCAN_BUFFER_SIZE + 1]; // buffer to record raw frame nibbles before parsing
    uint32_t _pending_frame_size = 0; // holds the size of frame to be tx
    int16_t pos_ = 0; // position in the buffer recording nibble frames before parsing
    AP_HAL::UARTDriver* _port; // UART interface port reference to be used for SLCAN iface

    ObjectBuffer<AP_HAL::CANIface::CanRxItem> rx_queue_; // Parsed Rx Frame queue

    const uint32_t _serial_lock_key = 0x53494442; // Key used to lock UART port for use by slcan
    
    AP_Int8 _slcan_can_port;
    AP_Int8 _slcan_ser_port;
    AP_Int8 _slcan_timeout;
    AP_Int8 _slcan_mode;

    AP_HAL::CANIface* _can_iface; // Can interface to be used for interaction by SLCAN interface

public:
    CANIface():
        rx_queue_(HAL_CAN_RX_QUEUE_SIZE)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    static const struct AP_Param::GroupInfo var_info[];

    bool init(const uint32_t bitrate, const OperatingMode mode) override
    {
        return false;
    }

    // Initialisation of SLCAN Passthrough method of operation
    bool init_passthrough(uint8_t i);

    // clears old frames from Rx Queue
    void clear_rx() override
    {
        rx_queue_.clear();
    }

    // Set UART port to be used with slcan interface
    int set_port(AP_HAL::UARTDriver* port);

    // check if the interface is initialised
    bool is_initialized() const override;

    void reset_params();

    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline,
                 AP_HAL::CANIface::CanIOFlags flags) override;
    
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& rx_time,
                            AP_HAL::CANIface::CanIOFlags& out_flags) override;
};

}

#endif