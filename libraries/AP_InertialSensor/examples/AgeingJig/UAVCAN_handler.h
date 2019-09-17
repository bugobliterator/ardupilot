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
    static void loop(void);
    void send_own_status_message();
    void forward_status_message(const uavcan::ReceivedDataStructure<com::hex::equipment::jig::Status>& msg);
private:
    uint8_t _interface;
    static int8_t _heater_target_temp;
    static uint16_t sensor_health_mask;
    static uint32_t _hold_start_ms;
    static uint8_t _heater_state;
    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
        SystemClock()
        {
        }

        uavcan::UtcDuration utc_adjustment;
        virtual void adjustUtc(uavcan::UtcDuration adjustment)
        {
            utc_adjustment = adjustment;
        }

    public:
        virtual uavcan::MonotonicTime getMonotonic() const
        {
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            return uavcan::MonotonicTime::fromUSec(usec);
        }
        virtual uavcan::UtcTime getUtc() const
        {
            uavcan::UtcTime utc;
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            utc.fromUSec(usec);
            utc += utc_adjustment;
            return utc;
        }

        static SystemClock& instance()
        {
            static SystemClock inst;
            return inst;
        }
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

    uavcan::HeapBasedPoolAllocator<UAVCAN_NODE_POOL_BLOCK_SIZE, UAVCAN_handler::RaiiSynchronizer> _node_allocator;
};
