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
    ScriptingCANBuffer *buffer = can_buffers[sensor_index].take_at_most([&sensor, buffer_len]() {
        return sensor.add_buffer(buffer_len);
    });
    if (buffer == nullptr || !CANBufferAccess::reuse(*buffer, buffer_len)) {
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
    const uint32_t timeout_us = can_write_timeout_argument(L, 3);
    lua_pushboolean(L, ud->write_frame(frame, timeout_us));
    return 1;
}
#endif  // AP_SCRIPTING_CAN_SENSOR_ENABLED

void can_new_session()
{
#if AP_SCRIPTING_CAN_SENSOR_ENABLED
    for (auto &buffers : can_buffers) {
        buffers.new_session();
    }
#endif
}

#if HAL_ENABLE_DRONECAN_DRIVERS

#define IFACE_ALL uint8_t(((1U<<(HAL_NUM_CAN_IFACES))-1U))

// generated (lua_remote_dispatch.cpp)
DroneCAN_Handle *check_DroneCAN_Handle(lua_State *L, int arg);
DroneCAN_Handle *new_DroneCAN_Handle(lua_State *L);

// DroneCAN_Handle's functions of AP_Scripting_helpers.cpp, ported; a friend of the class
struct DroneCANAccess {
    static int new_handle(lua_State *L);
    static int broadcast(lua_State *L);
    static int request(lua_State *L);
    static int check_message(lua_State *L);
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

    auto &iface = h->dc->get_canard_iface();
    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeBroadcast;
    transfer.data_type_signature = h->signature;
    transfer.data_type_id = h->data_type;
    transfer.inout_transfer_id = &h->transfer_id;
    transfer.priority = 0;
    transfer.payload = data;
    transfer.payload_len = data_length;
    transfer.iface_mask = IFACE_ALL;
#if CANARD_ENABLE_CANFD
    transfer.canfd = h->canfd;
#endif
    transfer.timeout_ms = 10;

    bool ok = iface.broadcast(transfer);
    lua_pushboolean(L, ok);

    return 1;
}

// handle:request(node, payload): DroneCAN_Handle::request()
int DroneCANAccess::request(lua_State *L)
{
    binding_argcheck(L, 3);

    DroneCAN_Handle *h = check_DroneCAN_Handle(L, 1);
    uint8_t target_node = luaL_checknumber(L, 2);

    size_t data_length;
    const void *data = luaL_checklstring(L, 3, &data_length);

    auto &iface = h->dc->get_canard_iface();

    Canard::Transfer transfer;

    transfer.transfer_type = CanardTransferTypeRequest;
    transfer.data_type_signature = h->signature;
    transfer.data_type_id = h->data_type;
    transfer.inout_transfer_id = &h->transfer_id;
    transfer.priority = 0;
    transfer.payload = data;
    transfer.payload_len = data_length;
    transfer.iface_mask = IFACE_ALL;
#if CANARD_ENABLE_CANFD
    transfer.canfd = h->canfd;
#endif
    transfer.timeout_ms = 10;

    WITH_SEMAPHORE(iface.get_sem_rx());

    if (h->subscriber != nullptr) {
        delete h->subscriber;
    }

    h->subscriber = NEW_NOTHROW DroneCAN_Handle::Subscriber(*h, CanardTransferTypeResponse);
    bool ok = h->subscriber != nullptr;

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
    // peek first, as in-process; a payload longer than one reply carries is dropped, giving nil
    // (recorded deviation: in-process it is returned)
    if (!dronecan_peek_payload(s->payloads, payload)) {
        return 0;
    }

    lua_pushlstring(L, (char *)payload.data, payload.length);

    s->payloads.pop();
    free(payload.data);

    lua_pushinteger(L, payload.node_id);
    *new_uint64_t(L) = payload.timestamp;
#if CANARD_ENABLE_CANFD
    lua_pushboolean(L, payload.canfd);
#else
    lua_pushboolean(L, false);
#endif

    if (s->trans_type == CanardTransferTypeResponse) {
        // request reply removes the subscriber
        WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());
        delete h->subscriber;
        h->subscriber = nullptr;
    }

    return 4;
}

// DroneCAN_Handle::__gc(): run when the M4 collects the handle (m4:release) or the session ends
void DroneCANAccess::release(void *object)
{
    DroneCAN_Handle *h = static_cast<DroneCAN_Handle *>(object);
    if (h->subscriber == nullptr) {
        return;
    }
    WITH_SEMAPHORE(h->dc->get_canard_iface().get_sem_rx());
    delete h->subscriber;
    h->subscriber = nullptr;
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

void DroneCAN_Handle__release(void *object)
{
    DroneCANAccess::release(object);
}

#endif  // HAL_ENABLE_DRONECAN_DRIVERS

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
