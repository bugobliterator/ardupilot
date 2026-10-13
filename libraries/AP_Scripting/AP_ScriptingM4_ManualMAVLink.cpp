/*
  The MAVLink and mission-queue manual bindings of lua_bindings.cpp, ported to the remote adapter
  (gen-bindings -r remote_ready_manual[]). Dot calls (mavlink.init(10, 1)) arrive with a nil self
  (ROW_DOT_ACCESS). The receive queue, the message filters, the blocked commands and the mission
  queue stay in AP_Scripting on the M7, which the vehicle's MAVLink and mission code fill (spec §6).

  Waits on the host thread (the 900 ms rule): mavlink_data.sem, held by GCS threads for one push
  in AP_Scripting::handle_message(); comm_chan_lock(chan), held by the channel's senders for one
  message; mavlink_command_block_list_sem, held by GCS threads for one walk of at most
  MAVLINK_BLOCKED_COMMANDS_PER_SESSION entries. The queue pops are lock-free.
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <GCS_MAVLink/GCS.h>
#include "AP_Scripting.h"
#include "AP_ScriptingM4_Quota.h"
#include "AP_ScriptingM4_Remote.h"

namespace ScriptingM4Remote {

// the blocks this session's scripts made (lua_mavlink_block_command)
static CommandBlocks command_blocks;

#if HAL_GCS_ENABLED
static_assert(MAVLINK_SCRIPTING_COMMAND == MAV_CMD_SCRIPTING, "the command the GCS stops scripting with");

// mavlink:init(queue_size, num_msgs): lua_mavlink_init(). The queue and the filter table are made
// once per boot and kept across sessions, as in-process: at most 25 messages and 25 ids
int lua_mavlink_init(lua_State *L)
{
    // M4: no fix_dot_access_never_add_another_call(): a dot call arrives with a nil self
    binding_argcheck(L, 3);
    // get the depth of receive queue
    const uint32_t queue_size = get_uint32(L, 2, 0, 25);
    // get number of msgs to accept
    const uint32_t num_msgs = get_uint32(L, 3, 0, 25);

    struct AP_Scripting::mavlink &data = AP::scripting()->mavlink_data;
    bool failed = false;
    {
        WITH_SEMAPHORE(data.sem);
        if (data.rx_buffer == nullptr) {
            data.rx_buffer = NEW_NOTHROW ObjectBuffer<struct AP_Scripting::mavlink_msg>(queue_size);
        }
        if (data.accept_msg_ids == nullptr) {
            data.accept_msg_ids = NEW_NOTHROW uint32_t[num_msgs];
            if (data.accept_msg_ids != nullptr) {
                data.accept_msg_ids_size = num_msgs;
                memset(data.accept_msg_ids, UINT32_MAX, sizeof(int) * num_msgs);
            }
        }
        if ((data.rx_buffer == nullptr) || (data.accept_msg_ids == nullptr)) {
            delete data.rx_buffer;
            delete[] data.accept_msg_ids;
            data.rx_buffer = nullptr;
            data.accept_msg_ids = nullptr;
            data.accept_msg_ids_size = 0;
            failed = true;
        }
    } // release semaphore here as luaL_error will NOT do that!

    if (failed) {
        return luaL_error(L, "out of memory");
    }

    return 0;
}

// mavlink:receive_chan(): lua_mavlink_receive_chan(). The message is the 291-byte (or wide-sysid)
// C struct, as in-process; it fits one reply with the channel and the uint32_t timestamp
int lua_mavlink_receive_chan(lua_State *L)
{
    binding_argcheck(L, 1);

    struct AP_Scripting::mavlink_msg msg;
    ObjectBuffer<struct AP_Scripting::mavlink_msg> *rx_buffer = AP::scripting()->mavlink_data.rx_buffer;

    if (rx_buffer == nullptr) {
        return luaL_error(L, "RX not initialized");
    }

    if (rx_buffer->pop(msg)) {
        lua_pushlstring(L, (char *)&msg.msg, sizeof(msg.msg));
        lua_pushinteger(L, msg.chan);
        *new_uint32_t(L) = msg.timestamp_ms;
        return 3;
    } else {
        // no MAVLink to handle, just return no results
        return 0;
    }
}

// mavlink:register_rx_msgid(msgid): lua_mavlink_register_rx_msgid()
int lua_mavlink_register_rx_msgid(lua_State *L)
{
    binding_argcheck(L, 2);

    const uint32_t msgid = get_uint32(L, 2, 0, (1 << 24) - 1);

    struct AP_Scripting::mavlink &data = AP::scripting()->mavlink_data;

    // check that we aren't currently watching this ID
    for (uint8_t i = 0; i < data.accept_msg_ids_size; i++) {
        if (data.accept_msg_ids[i] == msgid) {
            lua_pushboolean(L, false);
            return 1;
        }
    }

    int i = 0;
    for (i = 0; i < data.accept_msg_ids_size; i++) {
        if (data.accept_msg_ids[i] == UINT32_MAX) {
            break;
        }
    }

    if (i >= data.accept_msg_ids_size) {
        return luaL_error(L, "no registrations free");
    }

    {
        WITH_SEMAPHORE(data.sem);
        data.accept_msg_ids[i] = msgid;
    }

    lua_pushboolean(L, true);
    return 1;
}

// mavlink:send_chan(chan, msgid, packet, [target_sysid]): lua_mavlink_send_chan(). The packet is
// the call's copy, bounded by MAX_STRING; at most max_msg_len bytes of it are copied
int lua_mavlink_send_chan(lua_State *L)
{
    if (lua_gettop(L) != 5) {
        binding_argcheck(L, 4);
    }

    const mavlink_channel_t chan = (mavlink_channel_t)get_uint32(L, 2, 0, MAVLINK_COMM_NUM_BUFFERS - 1);

    // Check if the channel is valid
    if (chan >= gcs().num_gcs()) {
        // Return nil
        return 0;
    }

    const uint32_t msgid = get_uint32(L, 3, 0, (1 << 24) - 1);

    size_t packet_len;
    const char *packet = luaL_checklstring(L, 4, &packet_len);
    // M4: lua_isnoneornil(): the adapter's lua_type() is LUA_TNONE (-1) or LUA_TNIL (0)
    const bool have_target = lua_type(L, 5) > 0;
    const uint32_t target_sysid = have_target ? get_uint32(L, 5, 0, UINT32_MAX) : 0;

    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) {
        // M4: the cast: the adapter's luaL_error is format-checked
        return luaL_error(L, "Unknown MAVLink message ID (%d)", int(msgid));
    }
    if (have_target && !(entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)) {
        return luaL_argerror(L, 5, "Message has no target system field");
    }
    // M4: the MavTx quota, after the checks; over it the send fails, as with no room on the link
    if (!remote_quota_allow(L, ScriptingM4::QuotaClass::MavTx)) {
        lua_pushboolean(L, false);
        return 1;
    }

    // Lua strings are immutable. Pad omitted extension fields and update the
    // payload target together with the extended header, without changing the string.
    char payload[MAVLINK_MAX_PAYLOAD_LEN] {};
    memcpy(payload, packet, MIN(packet_len, size_t(entry->max_msg_len)));
    if (have_target) {
        payload[entry->target_system_ofs] = mavlink_msg_target_field(target_sysid);
    }

    WITH_SEMAPHORE(comm_chan_lock(chan));
    if (target_sysid > UINT8_MAX &&
        (mavlink_get_channel_status(chan)->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)) {
        lua_pushboolean(L, false);
        return 1;
    }
    if (comm_get_txspace(chan) >= (GCS_MAVLINK::packet_overhead_chan(chan) + entry->max_msg_len)) {
        _mav_finalize_message_chan_send_target(chan,
                                        entry->msgid,
                                        payload,
                                        entry->min_msg_len,
                                        entry->max_msg_len,
                                        entry->crc_extra,
                                        target_sysid);

        lua_pushboolean(L, true);
    } else {
        lua_pushboolean(L, false);
    }

    return 1;
}

// mavlink:block_command(id): lua_mavlink_block_command(), with the session cap and
// MAV_CMD_SCRIPTING kept unblocked (CommandBlocks)
int lua_mavlink_block_command(lua_State *L)
{
    binding_argcheck(L, 2);

    const uint16_t id = get_uint16_t(L, 2);

    // Check if ID is already registered
    if (AP::scripting()->is_handling_command(id)) {
        lua_pushboolean(L, true);
        return 1;
    }

    // M4: at most MAVLINK_BLOCKED_COMMANDS_PER_SESSION, and never MAV_CMD_SCRIPTING
    if (!command_blocks.allow(id)) {
        lua_pushboolean(L, false);
        return 1;
    }

    // Add new list item
    AP_Scripting::command_block_list *new_item = NEW_NOTHROW AP_Scripting::command_block_list;
    if (new_item == nullptr) {
        lua_pushboolean(L, false);
        return 1;
    }
    new_item->id = id;
    command_blocks.take();

    {
        WITH_SEMAPHORE(AP::scripting()->mavlink_command_block_list_sem);
        new_item->next = AP::scripting()->mavlink_command_block_list;
        AP::scripting()->mavlink_command_block_list = new_item;
    }

    lua_pushboolean(L, true);
    return 1;
}
#endif // HAL_GCS_ENABLED

#if AP_MISSION_ENABLED
// mission_receive(): lua_mission_receive()
int lua_mission_receive(lua_State *L)
{
    binding_argcheck(L, 0);

    ObjectBuffer<struct AP_Scripting::scripting_mission_cmd> *input = AP::scripting()->mission_data;

    if (input == nullptr) {
        // no mission items ever received
        return 0;
    }

    struct AP_Scripting::scripting_mission_cmd cmd;

    if (!input->pop(cmd)) {
        // no new item
        return 0;
    }

    *new_uint32_t(L) = cmd.time_ms;

    lua_pushinteger(L, cmd.p1);
    lua_pushnumber(L, cmd.content_p1);
    lua_pushnumber(L, cmd.content_p2);
    lua_pushnumber(L, cmd.content_p3);

    return 5;
}
#endif // AP_MISSION_ENABLED

void mavlink_end_session()
{
    command_blocks.new_session();
#if AP_MISSION_ENABLED
    // the session's unread mission items go with it (recorded deviation: in-process a restarted
    // script still receives them). The host thread is the queue's only consumer
    (void)drain_queue(AP::scripting()->mission_data);
#endif
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
