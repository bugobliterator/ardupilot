#include "UAVCAN_handler.h"
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#define debug_jig(fmt, args...) do { hal.console->printf(fmt, ##args); } while (0)

extern const AP_HAL::HAL &hal;

static UAVCAN_handler uavcan_handle[2];

static int selfid = -1;
static int max_id = 0;

static int _node_status_count[2], _jig_status_count[2];

static void cb_NodeStatus(const uavcan::ReceivedDataStructure<uavcan::protocol::NodeStatus>& msg) 
{
    if ((msg.getSrcNodeID() == 127) && selfid == -1) {
        selfid = 0;
    }
    if (msg.getSrcNodeID().get() > 9) {
        _node_status_count[0]++;
    } else {
        _node_status_count[1]++;
    }
}

static void cb_JigStatus(const uavcan::ReceivedDataStructure<com::hex::equipment::jig::Status>& msg)
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
    if (msg.getSrcNodeID().get() > 9) {
        _jig_status_count[0]++;
    } else {
        _jig_status_count[1]++;
    }
}

uint16_t UAVCAN_handler::_sensor_health_mask = 0x3FF;
uint8_t UAVCAN_handler::_heater_state;

void UAVCAN_handler::init()
{
    uavcan_handle[0].init(0);
    uavcan_handle[1].init(1);
}


void UAVCAN_handler::loop_all()
{
    uavcan_handle[0].loop();
    uavcan_handle[1].loop();
}


void UAVCAN_handler::loop(void)
{
    const int error = _node->spin(uavcan::MonotonicDuration::fromMSec(1));
    if (error < 0) {
        hal.scheduler->delay_microseconds(100);
        return;
    }
    // Send Messages
    if ((AP_HAL::millis() - last_msg_sent_time) > 1000) {
        last_msg_sent_time = AP_HAL::millis();
        if (selfid != -1) {
            send_own_status_message();
        }
        hal.console->printf("CAN%d NodeStatus: %d JigStatus: %d\n", _interface, _node_status_count[_interface] ,_jig_status_count[_interface]);
        hal.uartC->printf("CAN%d NodeStatus: %d JigStatus: %d\n", _interface, _node_status_count[_interface] ,_jig_status_count[_interface]);
        _node_status_count[_interface] = 0;
        _jig_status_count[_interface] = 0;
    }
}

void UAVCAN_handler::set_sensor_states(uint16_t sensor_health_mask, uint8_t heater_state)
{
    _sensor_health_mask = sensor_health_mask;
    _heater_state = heater_state;
}

void UAVCAN_handler::send_own_status_message()
{
    if (_node == nullptr) {
        return;
    }
    com::hex::equipment::jig::Status status;
    // loop and print each sensor
    status.id = selfid;
    status.max_id = max_id;
    status.heater_state = _heater_state;
    status.message_type = com::hex::equipment::jig::Status::MESSAGE_TYPE_PRIMARY;
    status.sensor_health_mask = _sensor_health_mask;
    status.temperature = AP::ins().get_temperature(0);
    _status_pub->broadcast(status);
}

void UAVCAN_handler::forward_status_message(const uavcan::ReceivedDataStructure<com::hex::equipment::jig::Status>& msg)
{
    if (_node == nullptr) {
        return;
    }
    com::hex::equipment::jig::Status status;
    hal.console->printf("Forwarding Jig Status %d\n", _node->getNodeID().get());
    hal.uartC->printf("Forwarding Jig Status %d\n", _node->getNodeID().get());
    // loop and print each sensor
    status.id = msg.id;
    status.max_id = msg.max_id;
    status.heater_state = msg.heater_state;
    status.message_type = com::hex::equipment::jig::Status::MESSAGE_TYPE_FORWARD;
    status.sensor_health_mask = msg.sensor_health_mask;
    status.temperature = msg.temperature;
    _status_pub->broadcast(status);
}

void UAVCAN_handler::init(uint8_t interface)
{
    AP_HAL::CANManager* can_mgr = new ChibiOS::CANManager;
    _interface = interface;
    if (can_mgr == nullptr) {
        AP_HAL::panic("Couldn't allocate CANManager, something is very wrong");
    }

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

    _status_pub = new uavcan::Publisher<com::hex::equipment::jig::Status>(*_node);
    _status_pub->setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));
    _status_pub->setPriority(uavcan::TransferPriority::OneLowerThanHighest);

#define START_CB(mtype, cbname) (new uavcan::Subscriber<mtype>(*_node))->start(cb_ ## cbname)

    START_CB(uavcan::protocol::NodeStatus, NodeStatus);
    START_CB(com::hex::equipment::jig::Status, JigStatus);

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
