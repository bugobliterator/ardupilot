#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL_ChibiOS/CAN.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <uavcan/uavcan.hpp>
#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <com/hex/equipment/jig/Status.hpp>
#include <uavcan/protocol/NodeStatus.hpp>

class UAVCAN_handler {
public:
    UAVCAN_handler();
    ~UAVCAN_handler();

    void init(uint8_t interface);
    static void init();
    void loop(void);
    static void loop_all(void);
    void send_own_status_message();
    void forward_status_message(const uavcan::ReceivedDataStructure<com::hex::equipment::jig::Status>& msg);
    static void set_sensor_states(uint16_t sensor_health_mask, uint8_t heater_state);
private:
    uint8_t _interface;
    char _thread_name[9];

    static uint16_t _sensor_health_mask;
    static uint8_t _heater_state;

    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
    public:
        SystemClock() = default;

        void adjustUtc(uavcan::UtcDuration adjustment) override {
            utc_adjustment_usec = adjustment.toUSec();
        }

        uavcan::MonotonicTime getMonotonic() const override {
            return uavcan::MonotonicTime::fromUSec(AP_HAL::micros64());
        }

        uavcan::UtcTime getUtc() const override {
            return uavcan::UtcTime::fromUSec(AP_HAL::micros64() + utc_adjustment_usec);
        }

        static SystemClock& instance() {
            static SystemClock inst;
            return inst;
        }

    private:
        int64_t utc_adjustment_usec;
    };

    uavcan::Node<0> *_node;

    uavcan::ISystemClock& get_system_clock();

    // This will be needed to implement if UAVCAN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {
    public:
        RaiiSynchronizer()
        {
        }

        ~RaiiSynchronizer()
        {
        }
    };
    uint32_t last_msg_sent_time;
    uavcan::Publisher<com::hex::equipment::jig::Status> *_status_pub;
    uavcan::HeapBasedPoolAllocator<UAVCAN_NODE_POOL_BLOCK_SIZE, UAVCAN_handler::RaiiSynchronizer> _node_allocator;
};
