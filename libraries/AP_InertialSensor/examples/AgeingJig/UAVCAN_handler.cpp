#include "UAVCAN_handler.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#define debug_jig(fmt, args...) do { hal.console->printf(fmt, ##args); } while (0)

#define IMU_HIGH_TEMP 65
#define IMU_LOW_TEMP  50

extern const AP_HAL::HAL &hal;

static UAVCAN_handler uavcan_handle[2];

static int selfid = -1;
static int max_id = 0;

static void nodestatus_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg) 
{
    if (msg.getSrcNodeID() == 127 && selfid == -1) {
        hal.console->printf("I am the Master!\n");
        selfid = 0;
        uavcan_handle[1].init(1);
    }
    uavcan_handle[0].send_own_status_message();
    uavcan_handle[1].send_own_status_message();
}

static void jigstatus_cb(const uavcan::ReceivedDataStructure<com::hex::equipment::jig::Status>& msg)
{
    uint8_t src_nodeid = msg.getSrcNodeID().get();

    if (src_nodeid == 10 && selfid == -1) {
        selfid = msg.id + 1;
        if (max_id == 0) {
            max_id = selfid;
        }
    } else if (src_nodeid ==  9) {
        max_id = msg.max_id;
        uavcan_handle[0].forward_status_message(msg);
    }
}

uint16_t UAVCAN_handler::sensor_health_mask = 0xFF;
int8_t UAVCAN_handler::_heater_target_temp = IMU_HIGH_TEMP;
uint32_t UAVCAN_handler::_hold_start_ms;
uint8_t UAVCAN_handler::_heater_state;

void UAVCAN_handler::init()
{
    uavcan_handle[0].init(0);
}


void UAVCAN_handler::loop(void)
{
    if (uavcan_handle[0]._node != nullptr) {
        uavcan_handle[0]._node->spin(uavcan::MonotonicDuration::fromMSec(1));
    }

    if (uavcan_handle[1]._node != nullptr) {
        uavcan_handle[1]._node->spin(uavcan::MonotonicDuration::fromMSec(1));
    }

    for (uint8_t i = 0; i < 3; i++) {
        if (!AP::ins().get_accel_health(i)) {
            sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << i);
        }
        if (!AP::ins().get_gyro_health(i)) {
            sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << i);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::baro().healthy(i)) {
            sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << i);
        }
    }

    if ((_heater_target_temp - AP::ins().get_temperature(0)) > 3.0f) {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_HEATING;
        _hold_start_ms = AP_HAL::millis();
    } else if ((_heater_target_temp - AP::ins().get_temperature(0)) < -3.0f) {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_COOLING;
        _hold_start_ms = AP_HAL::millis();
    } else {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_HOLDING;
    }
    //hal.console->printf("Temp Delta: %d %d %f\n", _heater_state, AP_HAL::millis() - _hold_start_ms, (_heater_target_temp - AP::ins().get_temperature(0)));

    if ((_heater_state == com::hex::equipment::jig::Status::HEATER_STATE_HOLDING) && ((AP_HAL::millis() - _hold_start_ms) >= 10000)) {
        if (_heater_target_temp == IMU_HIGH_TEMP) {
            _heater_target_temp = IMU_LOW_TEMP;
        } else if (_heater_target_temp == IMU_LOW_TEMP) {
            _heater_target_temp = IMU_HIGH_TEMP;
        }
    }

    hal.util->set_imu_target_temp(&_heater_target_temp);
}

void UAVCAN_handler::send_own_status_message()
{
    if (_node == nullptr) {
        return;
    }
    uavcan::Publisher<com::hex::equipment::jig::Status> status_pub(*_node);
    com::hex::equipment::jig::Status status;
    // loop and print each sensor
    status.id = selfid;
    status.max_id = max_id;
    status.heater_state = _heater_state;
    status.message_type = com::hex::equipment::jig::Status::MESSAGE_TYPE_PRIMARY;
    status.sensor_health_mask = sensor_health_mask;
    status.temperature = AP::ins().get_temperature(0);
    status_pub.broadcast(status);
}

void UAVCAN_handler::forward_status_message(const uavcan::ReceivedDataStructure<com::hex::equipment::jig::Status>& msg)
{
    if (_node == nullptr) {
        return;
    }
    uavcan::Publisher<com::hex::equipment::jig::Status> status_pub(*_node);
    com::hex::equipment::jig::Status status;
    // loop and print each sensor
    status.id = msg.id;
    status.max_id = msg.max_id;
    status.heater_state = msg.heater_state;
    status.message_type = com::hex::equipment::jig::Status::MESSAGE_TYPE_FORWARD;
    status.sensor_health_mask = msg.sensor_health_mask;
    status.temperature = msg.temperature;
    status_pub.broadcast(status);
}

void UAVCAN_handler::init(uint8_t interface)
{
    AP_HAL::CANManager* can_mgr = new ChibiOS::CANManager;
    
    if (can_mgr == nullptr) {
        AP_HAL::panic("Couldn't allocate CANManager, something is very wrong");
    }

    _interface = interface;
    const_cast <AP_HAL::HAL&> (hal).can_mgr[interface] = can_mgr;
    can_mgr->begin(1000000, interface);
    can_mgr->initialized(true);

    if (!can_mgr->is_initialized()) {
        debug_jig("Can not initialised\n");
        return;
    }

    uavcan::ICanDriver* driver = can_mgr->get_driver();
    if (driver == nullptr) {
        return;
    }

    _node = new uavcan::Node<0>(*driver, get_system_clock(), _node_allocator);

    if (_node == nullptr) {
        return;
    }

    if (_node->isStarted()) {
        return;
    }

    uavcan::NodeID self_node_id(9 + interface);
    _node->setNodeID(self_node_id);

    char ndname[20];
    snprintf(ndname, sizeof(ndname), "org.ardupilot:%u", interface);

    uavcan::NodeStatusProvider::NodeName name(ndname);
    _node->setName(name);

    uavcan::protocol::SoftwareVersion sw_version; // Standard type uavcan.protocol.SoftwareVersion
    sw_version.major = AP_UAVCAN_SW_VERS_MAJOR;
    sw_version.minor = AP_UAVCAN_SW_VERS_MINOR;
    _node->setSoftwareVersion(sw_version);

    uavcan::protocol::HardwareVersion hw_version; // Standard type uavcan.protocol.HardwareVersion

    hw_version.major = AP_UAVCAN_HW_VERS_MAJOR;
    hw_version.minor = AP_UAVCAN_HW_VERS_MINOR;
    _node->setHardwareVersion(hw_version);

    int start_res = _node->start();
    if (start_res < 0) {
        debug_jig("UAVCAN: node start problem\n\r");
        return;
    }
    (new uavcan::Subscriber<uavcan::protocol::NodeStatus>(*_node))->start(nodestatus_cb);
    (new uavcan::Subscriber<com::hex::equipment::jig::Status>(*_node))->start(jigstatus_cb);

    /*
     * Informing other nodes that we're ready to work.
     * Default mode is INITIALIZING.
     */
    _node->setModeOperational();

    debug_jig("UAVCAN: init done\n\r");
}

uavcan::ISystemClock & UAVCAN_handler::get_system_clock()
{
    return SystemClock::instance();
}

UAVCAN_handler::UAVCAN_handler() :
    _node_allocator(UAVCAN_NODE_POOL_SIZE, UAVCAN_NODE_POOL_SIZE)
{}

UAVCAN_handler::~UAVCAN_handler()
{
}
