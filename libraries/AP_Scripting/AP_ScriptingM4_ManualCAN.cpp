/*
  The CAN manual bindings, ported to the remote adapter (gen-bindings -r
  remote_ready_manual[]): the scripting CAN buffers of lua_bindings.cpp, and
  DroneCAN_Handle of AP_Scripting_helpers.cpp. A CAN buffer is a session
  object the scripting CAN sensor keeps, as in-process; an M4 session holds at
  most CAN_BUFFERS_PER_SESSION per sensor, and a later session is handed the
  same buffers again. A DroneCAN_Handle is an owned object in the M7's pool;
  its release is the in-process __gc. DroneCANAccess is a friend of
  DroneCAN_Handle, CANBufferAccess one of ScriptingCANBuffer.
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include "AP_Scripting.h"
#include "AP_Scripting_CANSensor.h"
#include "AP_Scripting_helpers.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Remote.h"

namespace ScriptingM4Remote {

#if AP_SCRIPTING_CAN_SENSOR_ENABLED
// generated (lua_remote_dispatch.cpp)
ScriptingCANBuffer **check_ScriptingCANBuffer(lua_State *L, int arg);
ScriptingCANBuffer **new_ScriptingCANBuffer(lua_State *L);
AP_HAL::CANFrame *check_AP_HAL__CANFrame(lua_State *L, int arg);

// the buffers M4 sessions were handed, of CAN:get_device() and CAN:get_device2()
static ScriptingM4::SessionRecycler<ScriptingCANBuffer, CAN_BUFFERS_PER_SESSION> can_buffers[2];

// a buffer of sensor for the call: one an earlier session was handed, else a new one, and nil
// past CAN_BUFFERS_PER_SESSION, before add_buffer() runs (recorded deviation: in-process adds
// buffers without bound)
static int push_CAN_buffer(lua_State *L, ScriptingCANSensor &sensor, uint8_t sensor_index, uint32_t buffer_len)
{
    // add_buffer() takes the sensor's semaphore blocking, as in-process: the CAN thread holds it
    // per frame, walking at most CAN_BUFFERS_PER_SESSION of this sensor's buffers
    ScriptingCANBuffer *buffer = take_CAN_buffer(can_buffers[sensor_index], [&sensor, buffer_len]() {
        return sensor.add_buffer(buffer_len);
    }, buffer_len);
    if (buffer == nullptr) {
        return 0;
    }
    // a handle refused after the buffer was taken takes another when the M4 retries; at most
    // CAN_BUFFERS_PER_SESSION, and they are handed out again in the next session
    *new_ScriptingCANBuffer(L) = buffer;
    return 1;
}

// CAN:get_device(buffer_len): lua_get_CAN_device(); a dot call (CAN.get_device) arrives with a nil self
int lua_get_CAN_device(lua_State *L)
{
    binding_argcheck(L, 2);

    const uint32_t raw_buffer_len = get_uint32(L, 2, 1, 25);
    const uint32_t buffer_len = static_cast<uint32_t>(raw_buffer_len);

    auto *scripting = AP::scripting();

    if (scripting->_CAN_dev == nullptr) {
        scripting->_CAN_dev = NEW_NOTHROW ScriptingCANSensor(AP_CAN::Protocol::Scripting);
        if (scripting->_CAN_dev == nullptr) {
            return luaL_argerror(L, 1, "CAN device nullptr");
        }
    }

    if (!scripting->_CAN_dev->initialized()) {
        // Driver not initialized, probably because there is no can driver set to scripting
        // Return nil
        return 0;
    }

    return push_CAN_buffer(L, *scripting->_CAN_dev, 0, buffer_len);
}

// CAN:get_device2(buffer_len): lua_get_CAN_device2()
int lua_get_CAN_device2(lua_State *L)
{
    binding_argcheck(L, 2);

    const uint32_t raw_buffer_len = get_uint32(L, 2, 1, 25);
    const uint32_t buffer_len = static_cast<uint32_t>(raw_buffer_len);

    auto *scripting = AP::scripting();

    if (scripting->_CAN_dev2 == nullptr) {
        scripting->_CAN_dev2 = NEW_NOTHROW ScriptingCANSensor(AP_CAN::Protocol::Scripting2);
        if (scripting->_CAN_dev2 == nullptr) {
            return luaL_argerror(L, 1, "CAN device nullptr");
        }
    }

    if (!scripting->_CAN_dev2->initialized()) {
        // Driver not initialized, probably because there is no can driver set to scripting 2
        // Return nil
        return 0;
    }

    return push_CAN_buffer(L, *scripting->_CAN_dev2, 1, buffer_len);
}

// buffer:write_frame(frame, timeout_us): the generated method with the timeout clamped to
// CAN_WRITE_TIMEOUT_MAX_US, the longest the host thread waits for the bus
int lua_m4_ScriptingCANBuffer_write_frame(lua_State *L)
{
    binding_argcheck(L, 3);
    ScriptingCANBuffer *ud = *check_ScriptingCANBuffer(L, 1);
    AP_HAL::CANFrame &frame = *check_AP_HAL__CANFrame(L, 2);
    return CAN_write_frame(L, *ud, frame, 3);
}
#endif  // AP_SCRIPTING_CAN_SENSOR_ENABLED

#if HAL_ENABLE_DRONECAN_DRIVERS

#define IFACE_ALL uint8_t(((1U<<(HAL_NUM_CAN_IFACES))-1U))

static_assert(DRONECAN_M4_TRANSFER_PRIORITY == CANARD_TRANSFER_PRIORITY_MEDIUM, "M4 transfers are medium priority");

// the subscribers of this session's handles (subscribe, request), at most DRONECAN_SUBSCRIBERS_PER_SESSION.
// Their changes take the canard rx semaphore blocking, as in-process: the DroneCAN thread holds it
// per received frame and per stale-transfer cleanup, and release() must not give up, as the
// subscriber would stay linked to a freed pool slot
static DroneCANSubscribers dronecan_subscribers;

// generated (lua_remote_dispatch.cpp)
DroneCAN_Handle *check_DroneCAN_Handle(lua_State *L, int arg);
DroneCAN_Handle *new_DroneCAN_Handle(lua_State *L);

// DroneCAN_Handle's functions of AP_Scripting_helpers.cpp, ported; a friend of the class
struct DroneCANAccess {
    static int new_handle(lua_State *L);
    static int broadcast(lua_State *L);
    static int request(lua_State *L);
    static int check_message(lua_State *L);
    static int subscribe(lua_State *L);
    static void release(void *object);
};

// DroneCAN_Handle(driver, signature, data_type [, canfd]): DroneCAN_Handle::new_handle(). A
// driver index outside 0..255 gives nil, as get_dronecan() does for a driver that is not
// there; in-process it is a float conversion out of range
int DroneCANAccess::new_handle(lua_State *L)
{
    lua_Number driver_index = luaL_checknumber(L, 1);
    const uint64_t sig = *check_uint64_t(L, 2);
    const uint16_t dtype = luaL_checkinteger(L, 3);
#if CANARD_ENABLE_CANFD
    const int args = lua_gettop(L);
    bool send_canfd = false;
    if (args > 3) {
        send_canfd = (bool)lua_toboolean(L, 4);
    }
#endif

    if (!(driver_index >= 0 && driver_index < 256)) {
        return 0;
    }
    auto *dc = AP_DroneCAN::get_dronecan(uint8_t(driver_index));
    if (dc == nullptr) {
        return 0;
    }

    auto *h = new_DroneCAN_Handle(L);

    h->dc = dc;
    h->signature = sig;
    h->data_type = dtype;
#if CANARD_ENABLE_CANFD
    h->canfd = send_canfd;
#endif

    return 1;
}

// handle:broadcast(payload): DroneCAN_Handle::broadcast(); the payload is the call's arena copy
int DroneCANAccess::broadcast(lua_State *L)
{
    binding_argcheck(L, 2);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);

    size_t data_length;
    const void *data = luaL_checklstring(L, 2, &data_length);

    // the Can quota counts the transfer's frames; over it false, as a full TX queue gives
    if (!can_transfer_allowed(L, data_length)) {
        lua_pushboolean(L, false);
        return 1;
    }

    auto &iface = h->dc->get_canard_iface();
    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeBroadcast;
    dronecan_fill_transfer(transfer, *h, data, data_length);
    transfer.iface_mask = IFACE_ALL;
#if CANARD_ENABLE_CANFD
    transfer.canfd = h->canfd;
#endif

    bool ok = iface.broadcast(transfer);
    lua_pushboolean(L, ok);

    return 1;
}

// handle:request(node, payload): DroneCAN_Handle::request()
int DroneCANAccess::request(lua_State *L)
{
    binding_argcheck(L, 3);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    const uint8_t target_node = dronecan_target_node(L, 2);

    size_t data_length;
    const void *data = luaL_checklstring(L, 3, &data_length);

    if (!can_transfer_allowed(L, data_length)) {
        lua_pushboolean(L, false);
        return 1;
    }

    auto &iface = h->dc->get_canard_iface();

    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeRequest;
    dronecan_fill_transfer(transfer, *h, data, data_length);
    transfer.iface_mask = IFACE_ALL;
#if CANARD_ENABLE_CANFD
    transfer.canfd = h->canfd;
#endif

    WITH_SEMAPHORE(iface.get_sem_rx());

    bool ok = dronecan_subscribers.replace(h->subscriber, [h]() {
        return NEW_NOTHROW DroneCAN_Handle::Subscriber(*h, CanardTransferTypeResponse);
    });

    if (ok) {
        h->subscriber->node_id = target_node;
        ok &= iface.request(target_node, transfer);
    }

    lua_pushboolean(L, ok);

    return 1;
}

// handle:check_message(): DroneCAN_Handle::check_message()
int DroneCANAccess::check_message(lua_State *L)
{
    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);

    auto *s = h->subscriber;

    if (s == nullptr) {
        return 0;
    }
    DroneCAN_Handle::Subscriber::Payload payload;
    // a payload longer than one reply carries is dropped, giving nil (recorded deviation:
    // in-process it is returned)
    if (!dronecan_push_payload(L, s->payloads, payload)) {
        return 0;
    }
#if CANARD_ENABLE_CANFD
    lua_pushboolean(L, payload.canfd);
#else
    lua_pushboolean(L, false);
#endif

    if (s->trans_type == CanardTransferTypeResponse) {
        // request reply removes the subscriber
        WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());
        dronecan_subscribers.drop(h->subscriber);
    }

    return 4;
}

// handle:subscribe(): DroneCAN_Handle::subscribe(), within the session's subscriber cap; false past it
int DroneCANAccess::subscribe(lua_State *L)
{
    binding_argcheck(L, 1);
    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());
    lua_pushboolean(L, dronecan_subscribers.replace(h->subscriber, [h]() {
        return NEW_NOTHROW DroneCAN_Handle::Subscriber(*h, CanardTransferTypeBroadcast);
    }));
    return 1;
}

// DroneCAN_Handle::__gc(): run when the M4 collects the handle (m4:release) or the session ends
void DroneCANAccess::release(void *object)
{
    DroneCAN_Handle *h = static_cast<DroneCAN_Handle *>(object);
    if (h->subscriber == nullptr) {
        return;
    }
    WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());
    dronecan_subscribers.drop(h->subscriber);
}

int DroneCAN_Handle__new_handle(lua_State *L)
{
    return DroneCANAccess::new_handle(L);
}

int DroneCAN_Handle__broadcast(lua_State *L)
{
    return DroneCANAccess::broadcast(L);
}

int DroneCAN_Handle__request(lua_State *L)
{
    return DroneCANAccess::request(L);
}

int DroneCAN_Handle__check_message(lua_State *L)
{
    return DroneCANAccess::check_message(L);
}

int lua_m4_DroneCAN_Handle_subscribe(lua_State *L)
{
    return DroneCANAccess::subscribe(L);
}

void DroneCAN_Handle__release(void *object)
{
    DroneCANAccess::release(object);
}

#endif  // HAL_ENABLE_DRONECAN_DRIVERS

void can_new_session()
{
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    for (auto &buffers : can_buffers) {
        buffers.new_session();
    }
#endif
#if HAL_ENABLE_DRONECAN_DRIVERS
    dronecan_subscribers.new_session();
#endif
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
