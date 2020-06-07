
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#if HAL_NUM_CAN_IFACES > 1
#include "AP_CANManager.h"
#include "AP_CANTester.h"
#include "AP_CANManager.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <AP_ToshibaCAN/AP_ToshibaCAN.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo CANTester::var_info[] = {
    // @Param: ID
    // @DisplayName: CAN Test Index
    // @Description: Selects the Index of Test that needs to be run recursively, this value gets reset to 0 at boot.
    // @Range: 0 4
    // @Values: 0: TEST_NONE, 1: TEST_LOOPBACK,2: TEST_BUSOFF_RECOVERY,3: TEST_UAVCAN_DNA,4: TEST_TOSHIBA_CAN
    // @User: Advanced
    AP_GROUPINFO("ID", 1, CANTester, _test_id, 0),
    // @Param: LPR8
    // @DisplayName: CANTester LoopRate
    // @Description: Selects the Looprate of Test methods
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("LPR8", 2, CANTester, _loop_rate, 1000),

    AP_GROUPEND

};

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "TestCAN",  fmt, #args); } while (0)

bool CANTester::add_interface(AP_HAL::CANIface* can_iface)
{
    if (_num_ifaces >= HAL_NUM_CAN_IFACES) {
        debug_can(AP_CANManager::LOG_ERROR, "Max Number of CanIfaces exceeded");
    }

    _can_ifaces[_num_ifaces] = can_iface;

    if (_can_ifaces[_num_ifaces] == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CAN driver not found");
        return false;
    }

    if (!_can_ifaces[_num_ifaces]->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "Driver not initialized");
        return false;
    }

    _num_ifaces++;
    return true;
}

void CANTester::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    // Reset Test mask
    _test_id.set_and_save(0);

    debug_can(AP_CANManager::LOG_DEBUG, "starting init");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "already initialized");
        return;
    }

    if (_can_ifaces[0] == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "Interface not found");
        return;
    }

    // kick start tester thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&CANTester::main_thread, void), "can_tester", 4096, AP_HAL::Scheduler::PRIORITY_CAN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "couldn't create thread");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "init done");

    return;
}

// write frame on CAN bus
bool CANTester::write_frame(uint8_t iface, AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    if (!_can_ifaces[iface]->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "Cannot add event handle");
        return false;
    }
    // wait for space in buffer to send command
    bool read_select = false;
    bool write_select = true;
    out_frame.id += iface;
    int ret = _can_ifaces[iface]->select(read_select, write_select, &out_frame, AP_HAL::native_micros64() + timeout);
    if (ret < 0 || !write_select) {
        // delay if no space is available to send
        return false;
    }
    // send frame and return success
    return (_can_ifaces[iface]->send(out_frame, AP_HAL::native_micros64() + 100000, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool CANTester::read_frame(uint8_t iface, AP_HAL::CANFrame &recv_frame, uint64_t timeout, AP_HAL::CANIface::CanIOFlags &flags)
{
    if (!_can_ifaces[iface]->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "Cannot add event handle");
        return false;
    }
    // wait for space in buffer to read
    bool read_select = true;
    bool write_select = false;
    int ret = _can_ifaces[iface]->select(read_select, write_select, nullptr, AP_HAL::native_micros64() + timeout);
    if (ret < 0 || !read_select) {
        // return false if no data is available to read
        return false;
    }
    uint64_t time;

    // read frame and return success
    return (_can_ifaces[iface]->receive(recv_frame, time, flags) == 1);
}

void CANTester::main_thread()
{
    while (true) {
        switch (_test_id) {
        case CANTester::TEST_LOOPBACK:
            if (_can_ifaces[1] != nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running Loopback Test*******");
                if (test_loopback(_loop_rate)) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Loopback Test Pass*******");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Loopback Test Fail*******");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Can't do Loopback Test with single iface");
            }
            break;
        case CANTester::TEST_BUSOFF_RECOVERY:
            if (_can_ifaces[1] != nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running Busoff Recovery Test********");
                if (test_busoff_recovery()) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Busoff Recovery Test Pass********");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Busoff Recovery Test Fail********");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Can't do Busoff Recovery Test with single iface");
            }
            break;
        case CANTester::TEST_UAVCAN_DNA:
            if (_can_ifaces[1] == nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running UAVCAN DNA Test********");
                if (test_uavcan_dna()) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********UAVCAN DNA Test Pass********");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********UAVCAN DNA Test Fail********");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Only one iface needs to be set for UAVCAN_DNA_TEST");
            }
            break;
        case CANTester::TEST_TOSHIBA_CAN:
            if (_can_ifaces[1] == nullptr) {
                gcs().send_text(MAV_SEVERITY_ALERT, "********Running Toshiba CAN Test********");
                if (test_toshiba_can()) {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Toshiba CAN Test Pass********");
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "********Toshiba CAN Test Fail********");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ALERT, "Only one iface needs to be set for UAVCAN_DNA_TEST");
            }
            break;
        default:
            break;
        }

        for (uint8_t i = 0; i < 2; i++) {
            if (_can_ifaces[i] != nullptr) {
                _can_ifaces[i]->flush_tx();
            }
        }
        hal.scheduler->delay(5000);
        for (uint8_t i = 0; i < 2; i++) {
            if (_can_ifaces[i] != nullptr) {
                _can_ifaces[i]->clear_rx();
            }
        }
    }
}

/*****************************************
 *           Loopback Test               *
 * ***************************************/

#define NUM_LOOPBACK_RUNS 1000UL
#define LOOPBACK_MAGIC 0x34567819UL
#define NUM_MAX_TX_FRAMES 64
bool CANTester::test_loopback(uint32_t loop_rate)
{
    AP_HAL::CANFrame frame;
    AP_HAL::CANIface::CanIOFlags flags;
    uint32_t num_loops = NUM_LOOPBACK_RUNS;
    memset(&_loopback_stats[0], 0, sizeof(_loopback_stats[0]));
    memset(&_loopback_stats[1], 0, sizeof(_loopback_stats[1]));

    while (num_loops--) {
        for (uint8_t i = 0; i < 2; i++) {
            // Write as many frames as we can on an iface
            for (uint32_t tx_frames = 0; tx_frames < NUM_MAX_TX_FRAMES; tx_frames++) {
                create_loopback_frame(_loopback_stats[i], frame);
                if (write_frame(i, frame, 0)) {
                    _loopback_stats[i].tx_seq++;
                    _loopback_stats[i].num_tx++;
                } else {
                    break;
                }
            }

            // Also read as much as we can from the second iface
            while(true) {
                reset_frame(frame);
                if (read_frame((i+1)%2, frame, 0, flags)) {
                    if (frame.id != ((13 | AP_HAL::CANFrame::FlagEFF) + i)) {
                        continue;
                    }
                    check_loopback_frame(_loopback_stats[i], frame);
                    _loopback_stats[i].num_rx++;
                } else {
                    break;
                }
            }
        }
        hal.scheduler->delay_microseconds(loop_rate);
    }
    _can_ifaces[0]->flush_tx();
    _can_ifaces[1]->flush_tx();
    hal.scheduler->delay_microseconds(5000);
    // flush the rx data still buffered in the interface
    for (uint8_t i = 0; i < 2; i++) {
        while(true) {
            reset_frame(frame);
            if (read_frame((i+1)%2, frame, 0, flags)) {
                if (frame.id != ((13 | AP_HAL::CANFrame::FlagEFF) + i)) {
                    continue;
                }
                check_loopback_frame(_loopback_stats[i], frame);
                _loopback_stats[i].num_rx++;
            } else {
                break;
            }
        }
    }

    for (uint8_t i = 0; i < _num_ifaces; i++) {
        hal.console->printf("Loopback Test Results %d->%d:\n", i, (i+1)%2);
        hal.console->printf("num_tx: %lu, failed_tx: %lu\n",
                        (long unsigned int)_loopback_stats[i].num_tx, (long unsigned int)_loopback_stats[i].failed_tx);
        hal.console->printf("num_rx: %lu, bad_rx_data: %lu, bad_rx_seq: %lu\n",
                        (long unsigned int)_loopback_stats[i].num_rx, (long unsigned int)_loopback_stats[i].bad_rx_data, (long unsigned int)_loopback_stats[i].bad_rx_seq);
        if (_loopback_stats[i].num_rx < 0.9f * _loopback_stats[i].num_tx ||
            _loopback_stats[i].failed_tx || _loopback_stats[i].bad_rx_seq ||
            _loopback_stats[i].bad_rx_data) {
            return false;
        }
    }
    return true;
}

void CANTester::reset_frame(AP_HAL::CANFrame& frame)
{
    frame.id = 0;
    memset(frame.data, 0, sizeof(frame.data));
    frame.dlc = 0;

}

void CANTester::create_loopback_frame(CANTester::loopback_stats_s &stats, AP_HAL::CANFrame& frame)
{
    frame.id = 13 | AP_HAL::CANFrame::FlagEFF;
    frame.dlc = AP_HAL::CANFrame::MaxDataLen;
    memcpy(frame.data, &stats.tx_seq, sizeof(stats.tx_seq));
    uint32_t loopback_magic = LOOPBACK_MAGIC;
    memcpy(&frame.data[4], &loopback_magic, sizeof(loopback_magic));
}


void CANTester::check_loopback_frame(CANTester::loopback_stats_s &stats, AP_HAL::CANFrame& frame)
{
    if (frame.dlc != AP_HAL::CANFrame::MaxDataLen) {
        stats.bad_rx_data++;
    }

    uint32_t loopback_magic = LOOPBACK_MAGIC;
    if (memcmp(&frame.data[4], &loopback_magic, sizeof(loopback_magic)) != 0) {
        stats.bad_rx_data++;
        return;
    }

    uint16_t curr_seq;

    memcpy(&curr_seq, frame.data, sizeof(curr_seq));

    if (stats.next_valid_seq != curr_seq) {
        stats.bad_rx_seq++;
    }
    stats.next_valid_seq  = curr_seq + 1;
}

/*****************************************
 *         Busoff Recovery Test          *
 * ***************************************/
bool CANTester::test_busoff_recovery()
{
    uint32_t num_busoff_runs = 100000;
    uint64_t timestamp;
    AP_HAL::CANIface::CanIOFlags flags;
    AP_HAL::CANFrame bo_frame;
    bo_frame.id = (12 | AP_HAL::CANFrame::FlagEFF);
    memset(bo_frame.data, 0xA, sizeof(bo_frame.data));
    bo_frame.dlc = AP_HAL::CANFrame::MaxDataLen;
    bool bus_off_detected = false;
    // Bus Fault can be introduced by shorting CANH and CANL
    gcs().send_text(MAV_SEVERITY_ERROR, "Introduce Bus Off Fault on the bus.");
    while (num_busoff_runs--) {
        if (bus_off_detected) {
            break;
        }
        //Spam the bus with same frame
        _can_ifaces[0]->send(bo_frame, AP_HAL::native_micros64()+1000, 0);
        _can_ifaces[1]->receive(bo_frame, timestamp, flags);
        _can_ifaces[1]->send(bo_frame, AP_HAL::native_micros64()+1000, 0);
        _can_ifaces[0]->receive(bo_frame, timestamp, flags);
        bus_off_detected = _can_ifaces[0]->is_busoff() || _can_ifaces[1]->is_busoff();
        hal.scheduler->delay_microseconds(50);
    }
    if (!bus_off_detected) {
        gcs().send_text(MAV_SEVERITY_ERROR, "BusOff not detected on the bus");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_ERROR, "BusOff detected remove Fault.");
    hal.scheduler->delay(1000);
    gcs().send_text(MAV_SEVERITY_ERROR, "Running Loopback test.");
    //Send Dummy Frames to clear the error
    _can_ifaces[0]->send(bo_frame, AP_HAL::native_micros64(), 0);
    bo_frame.id += 1;
    _can_ifaces[1]->send(bo_frame, AP_HAL::native_micros64(), 0);
    //Clear the CAN bus Rx Buffer
    hal.scheduler->delay(1000);
    _can_ifaces[0]->clear_rx();
    _can_ifaces[1]->clear_rx();

    return test_loopback(_loop_rate);
}

/*****************************************
 *             UAVCAN DNA Test           *
 * ***************************************/

bool CANTester::test_uavcan_dna()
{

    uavcan::CanIfaceMgr _uavcan_iface_mgr {};

    if (!_uavcan_iface_mgr.add_interface(_can_ifaces[0])) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to add iface");
        return false;
    }

    uavcan::Node<0> node(_uavcan_iface_mgr, uavcan::SystemClock::instance(), _node_allocator);
    node.setName("org.ardupilot.dnatest");

    uavcan::protocol::HardwareVersion hw_version;
    const uint8_t uid_buf_len = hw_version.unique_id.capacity();
    uint8_t uid_len = uid_buf_len;
    uint8_t unique_id[uid_buf_len];

    if (hal.util->get_system_id_unformatted(unique_id, uid_len)) {
        unique_id[uid_len - 1] -= 5;
        uavcan::copy(unique_id, unique_id + uid_len, hw_version.unique_id.begin());
    }

    node.setHardwareVersion(hw_version); // Copying the value to the node's internals

    /*
     * Starting the node normally, in passive mode (i.e. without node ID assigned).
     */
    const int node_start_res = node.start();
    if (node_start_res < 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Failed to start the node");
        return false;
    }

    /*
     * Initializing the dynamic node ID allocation client.
     * By default, the client will use TransferPriority::OneHigherThanLowest for communications with the allocator;
     * this can be overriden through the third argument to the start() method.
     */
    uavcan::DynamicNodeIDClient client(node);
    int expected_node_id = 100;
    int client_start_res = client.start(node.getHardwareVersion().unique_id,    // USING THE SAME UNIQUE ID AS ABOVE
                                        expected_node_id);
    if (client_start_res < 0) {
        gcs().send_text(MAV_SEVERITY_ALERT,"Failed to start the dynamic node");
    }

    /*
     * Waiting for the client to obtain for us a node ID.
     * This may take a few seconds.
     */
    gcs().send_text(MAV_SEVERITY_ALERT, "Allocation is in progress");
    uint32_t num_runs = 100;
    while (!client.isAllocationComplete() && num_runs--) {
        const int res = node.spin(uavcan::MonotonicDuration::fromMSec(200));    // Spin duration doesn't matter
        if (res < 0) {
            gcs().send_text(MAV_SEVERITY_ALERT, "Transient failure");
        }
    }
    gcs().send_text(MAV_SEVERITY_ALERT, "Dynamic NodeID %d allocated node ID %d",
                    int(client.getAllocatedNodeID().get()),
                    int(client.getAllocatorNodeID().get()));
    if (client.getAllocatedNodeID().get() != expected_node_id) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Unexpected Node Id, expected %d", expected_node_id);
        return false;
    }
    return true;
}

/*****************************************
 *            TOSHIBA CAN Test           *
 *****************************************/
#define NUM_TOSHIBA_TEST_RUN 10000
bool CANTester::test_toshiba_can()
{
    AP_HAL::CANFrame frame;
    uint16_t num_runs = NUM_TOSHIBA_TEST_RUN;
    uint32_t num_errors = 0;
    uint32_t num_lock_cmds = 0;
    uint32_t num_request_data_cmds = 0;
    uint32_t num_motor_cmds = 0;
    uint32_t start_time = AP_HAL::native_millis();
    AP_HAL::CANIface::CanIOFlags flags;

    while (num_runs--) {
        if (!read_frame(0, frame, _loop_rate, flags)) {
            continue;
        }
        if (flags & AP_HAL::CANIface::Loopback) {
            continue;
        }
        switch (frame.id) {
        case AP_ToshibaCAN::COMMAND_LOCK: {
            AP_ToshibaCAN::motor_lock_cmd_t lock_frame;
            if (sizeof(lock_frame) != frame.dlc) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Bad lock command length");
                num_errors++;
            }
            memcpy(&lock_frame, frame.data, sizeof(lock_frame));
            if (lock_frame.motor1 != 1 && lock_frame.motor1 != 2 &&
                lock_frame.motor2 != 1 && lock_frame.motor2 != 2 &&
                lock_frame.motor3 != 1 && lock_frame.motor3 != 2 &&
                lock_frame.motor4 != 1 && lock_frame.motor4 != 2 &&
                lock_frame.motor5 != 1 && lock_frame.motor5 != 2 &&
                lock_frame.motor6 != 1 && lock_frame.motor6 != 2 &&
                lock_frame.motor7 != 1 && lock_frame.motor7 != 2 &&
                lock_frame.motor8 != 1 && lock_frame.motor8 != 2 &&
                lock_frame.motor9 != 1 && lock_frame.motor9 != 2 &&
                lock_frame.motor10 != 1 && lock_frame.motor10 != 2 &&
                lock_frame.motor11 != 1 && lock_frame.motor11 != 2 &&
                lock_frame.motor12 != 1 && lock_frame.motor12 != 2) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Bad lock frame received!");
                num_errors++;
            }
            num_lock_cmds++;
            break;
        }
        case AP_ToshibaCAN::COMMAND_REQUEST_DATA: {
            AP_ToshibaCAN::motor_request_data_cmd_t request_data_cmd;
            if (sizeof(request_data_cmd) != frame.dlc) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Bad request data command length");
                num_errors++;
            }
            memcpy(&request_data_cmd, frame.data, sizeof(request_data_cmd));
            if ((request_data_cmd.motor1  != request_data_cmd.motor2 &&
                 request_data_cmd.motor2  != request_data_cmd.motor3 &&
                 request_data_cmd.motor3  != request_data_cmd.motor4 &&
                 request_data_cmd.motor4  != request_data_cmd.motor5 &&
                 request_data_cmd.motor5  != request_data_cmd.motor6 &&
                 request_data_cmd.motor6  != request_data_cmd.motor7 &&
                 request_data_cmd.motor7  != request_data_cmd.motor8 &&
                 request_data_cmd.motor8  != request_data_cmd.motor9 &&
                 request_data_cmd.motor9  != request_data_cmd.motor10 &&
                 request_data_cmd.motor10 != request_data_cmd.motor11 &&
                 request_data_cmd.motor11 != request_data_cmd.motor12 &&
                 request_data_cmd.motor12) &&
                (request_data_cmd.motor1 != 0 &&
                 request_data_cmd.motor1 != 1 &&
                 request_data_cmd.motor1 != 2 &&
                 request_data_cmd.motor1 != 3 &&
                 request_data_cmd.motor1 != 4 &&
                 request_data_cmd.motor1 != 5)) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Bad request frame received!");
                num_errors++;
            }
            num_request_data_cmds++;
            send_toshiba_can_reply(request_data_cmd.motor1);
            break;
        }
        case AP_ToshibaCAN::COMMAND_MOTOR1:
        case AP_ToshibaCAN::COMMAND_MOTOR2:
        case AP_ToshibaCAN::COMMAND_MOTOR3: {
            AP_ToshibaCAN::motor_rotation_cmd_t rotation_cmd;
            if (frame.dlc != sizeof(rotation_cmd)) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Bad motor command length");
                num_errors++;
            }
            memcpy(&rotation_cmd, frame.data, sizeof(rotation_cmd));
            if ((rotation_cmd.motor1 > AP_ToshibaCAN::TOSHIBACAN_OUTPUT_MAX) ||
                (rotation_cmd.motor2 > AP_ToshibaCAN::TOSHIBACAN_OUTPUT_MAX) ||
                (rotation_cmd.motor3 > AP_ToshibaCAN::TOSHIBACAN_OUTPUT_MAX) ||
                (rotation_cmd.motor4 > AP_ToshibaCAN::TOSHIBACAN_OUTPUT_MAX)) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Bad motor command data");
                num_errors++;
            }
            num_motor_cmds++;
            break;
        }
        default: {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Unsupported Command");
            num_errors++;
            break;
        }
        }
    }
    uint32_t num_secs = (AP_HAL::native_millis() - start_time)/1000;
    gcs().send_text(MAV_SEVERITY_ALERT, "Num Errors: %lu Cmds Lock: %lu Request: %lu Motor: %lu",
                    (long unsigned int)num_errors,
                    (long unsigned int)num_lock_cmds,
                    (long unsigned int)num_request_data_cmds,
                    (long unsigned int)num_motor_cmds);

    gcs().send_text(MAV_SEVERITY_ALERT, "Rates Lock: %lu Request: %lu Motor: %lu",
                    (long unsigned int)num_lock_cmds/num_secs,
                    (long unsigned int)num_request_data_cmds/num_secs,
                    (long unsigned int)num_motor_cmds/num_secs);
    if (num_errors) {
        return false;
    } else {
        return true;
    }
}

bool CANTester::send_toshiba_can_reply(uint32_t cmd)
{
    AP_HAL::CANFrame send_frame;
    for (uint8_t sub_id = 0; sub_id < TOSHIBACAN_MAX_NUM_ESCS; sub_id++) {
        // decode rpm and voltage data
        switch (cmd) {
        case 1: {
            // copy contents to our structure
            AP_ToshibaCAN::motor_reply_data1_t reply_data;
            reply_data.rpm = htobe16(100);
            reply_data.current_ma = htobe16(1000);
            reply_data.voltage_mv = htobe16(12000);
            memcpy(send_frame.data, &reply_data, sizeof(reply_data.data));
            send_frame.id = AP_ToshibaCAN::MOTOR_DATA1 + sub_id;
            memcpy(send_frame.data, &reply_data, sizeof(reply_data));
            send_frame.dlc = 8;
            write_frame(0, send_frame, 1000);
            continue;
        }

        // decode temperature data
        case 2: {
            // motor data2 data format is 8 bytes (64 bits)
            //    10 bits: U temperature
            //    10 bits: V temperature
            //    10 bits: W temperature
            //    10 bits: motor temperature
            //    remaining 24 bits: reserved
            const uint16_t temp = (300 * 5) + 20;
            send_frame.data[0] = (temp >> 2) & 0xFF;
            send_frame.data[1] = ((temp << 6) | ((temp >> 4) & 0x3F)) & 0xFF;
            send_frame.data[2] = (((temp << 4) & 0xF0) | ((temp >> 6) & 0x0F)) & 0xFF;
            send_frame.data[3] = (((temp << 2) & 0xFC) | ((temp >> 8) & 0x03)) & 0xFF;
            send_frame.data[4] = temp & 0xFF;
            send_frame.id = AP_ToshibaCAN::MOTOR_DATA2 + sub_id;
            send_frame.dlc = 8;
            write_frame(0, send_frame, 1000);
            continue;
        }

        // encode cumulative usage data
        case 3: {
            // motor data3 data format is 8 bytes (64 bits)
            //    3 bytes: usage in seconds
            //    2 bytes: number of times rotors started and stopped
            //    3 bytes: reserved
            uint32_t secs = AP_HAL::native_millis()/1000;
            send_frame.data[0] = (secs >> 16) & 0xFF;
            send_frame.data[0] = (secs >> 8) & 0xFF;
            send_frame.data[0] = (secs & 0xFF);
            send_frame.id = AP_ToshibaCAN::MOTOR_DATA3 + sub_id;
            send_frame.dlc = 8;
            write_frame(0, send_frame, 1000);
            continue;
        }
        default:
            return false;
        }
    }
    return true;
}
#endif //#if HAL_NUM_CAN_IFACES > 1
