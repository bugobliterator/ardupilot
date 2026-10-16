#include "m4_service.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <CubePilotFW/PeerHeader.h>
#include <AP_ScriptingM4_Bindings.h>
#include <AP_ScriptingM4_Layout.h>
#include <AP_ScriptingM4_Rpc.h>

namespace ScriptingM4 {

using CubePilotFW::Channel;
namespace PeerHeader = CubePilotFW::PeerHeader;

void M4Service::init(Channel &channel, uint8_t *window, LuaLink &link, const M4Platform &platform)
{
    _channel = &channel;
    _window = window;
    _link = &link;
    _platform = platform;
    if (_platform.cycles_per_us == 0) {
        _platform.cycles_per_us = 1;
    }
    memset(_stats, 0, sizeof(_stats));
    _seq = 0;
    memset(&_untimed, 0, sizeof(_untimed));
    _ready = false;
    _rx_stalled = false;
    _flooding = false;
    _call_timed = false;
    _last_stats_ms = platform.millis();
}

void M4Service::step()
{
    // a reply is drained under the lock forward_posted() holds from send to Sent: it is never
    // seen before its call is Sent and dropped as stale
    _platform.lock();
    step_locked();
    _platform.unlock();
}

bool M4Service::forward_posted()
{
    _platform.lock();
    if (_ready && _link->state.load() == LuaLink::Posted) {
        if (!_call_timed) {
            _call_timed = true;
            _call_posted_ms = _platform.millis();
        }
        forward_call();
    }
    const bool sent = _link->state.load() != LuaLink::Posted;
    _platform.unlock();
    return sent;
}

void M4Service::step_locked()
{
    drain();
    const uint32_t now_ms = _platform.millis();
    const uint8_t state = _link->state.load();
    if (state == LuaLink::Posted && !_call_timed) {
        // the call timeout runs from posting: waiting for Ready or ring space counts too
        _call_timed = true;
        _call_posted_ms = now_ms;
    }
    if ((state == LuaLink::Posted || state == LuaLink::Sent) && now_ms - _call_posted_ms > CALL_TIMEOUT_MS) {
        if (state == LuaLink::Sent) {
            MethodStats *stats = stats_for(_call_method);
            if (stats != nullptr) {
                stats->lost++;
            } else {
                _untimed.lost++;
            }
        }
        complete(-ETIMEDOUT);
    } else if (_ready && state == LuaLink::Posted) {
        // calls wait for the M7's Ready: frames sent before its re-handshake would be dropped
        forward_call();
    }
    if (_flooding) {
        if (now_ms - _flood_until_ms < 0x80000000U) {
            _flooding = false;   // FLOOD_MS has passed
        } else {
            flood();
        }
    }
    if (now_ms - _last_stats_ms >= 1000) {
        send_stats(now_ms);
    }
    // a receive error other than empty leaves the frame queued: stop beating so
    // the M7 sees a stale heartbeat and restarts us, which clears the ring
    if (!_rx_stalled) {
        PeerHeader::beat(_window, PeerHeader::Core::M4);
    }
}

void M4Service::drain()
{
    // bounded, so a flooding M7 cannot keep the M4 from beating
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t len = 0;
        const int ret = _channel->receive(_rx, sizeof(_rx), &len);
        if (ret != 0) {
            if (ret != -EAGAIN) {
                _rx_stalled = true;   // e.g. -EMSGSIZE or -EBADMSG: the frame is not consumed
            }
            return;
        }
        handle(_rx, len);
    }
}

void M4Service::handle(const uint8_t *buf, uint16_t len)
{
    MsgHeader header;
    if (!parse_header(buf, len, header)) {
        return;
    }
    switch (MsgType(header.type)) {
    case MsgType::Reply:
        handle_reply(buf, len, header);
        return;
    case MsgType::Ready:
        _ready = true;
        return;
    case MsgType::FaultInject:
        handle_fault_inject(buf);
        return;
    case MsgType::Call:
    case MsgType::Notify:
    case MsgType::Stats:
        return;   // M4 -> M7 only
    }
}

void M4Service::handle_reply(const uint8_t *buf, uint16_t len, const MsgHeader &header)
{
    if (_link->state.load() != LuaLink::Sent || header.seq != _call_seq) {
        return;   // reply to a call that already timed out
    }
    const uint32_t round_trip_us = (_platform.cycles() - _call_sent_cycles) / _platform.cycles_per_us;
    ReplyHeader reply;
    memcpy(&reply, buf, sizeof(reply));
    MethodStats *stats = stats_for(_call_method);
    if (stats != nullptr) {
        stats->round_trips.add(round_trip_us);
        stats->total_us += round_trip_us;
        const uint32_t m7_us = reply.m7_wake_us + reply.m7_exec_us;
        if (m7_us > stats->m7_max_us) {
            stats->m7_max_us = m7_us;
        }
    } else {
        _untimed.count++;
        _untimed.total_us += round_trip_us;
        const uint32_t m7_us = reply.m7_wake_us + reply.m7_exec_us;
        // no histogram: the percentiles of these calls are bounded by their largest
        if (round_trip_us > _untimed.max_us) {
            _untimed.p50_us = _untimed.p99_us = _untimed.max_us = round_trip_us;
        }
        if (m7_us > _untimed.m7_max_us) {
            _untimed.m7_max_us = m7_us;
        }
    }
    memcpy(_link->reply, buf, len);
    complete(len);
}

void M4Service::handle_fault_inject(const uint8_t *buf)
{
    FaultInjectMsg inject;
    memcpy(&inject, buf, sizeof(inject));
    if (inject.kind == uint32_t(FaultKind::None) || inject.kind > FAULT_KIND_MAX) {
        return;
    }
    const FaultKind kind = FaultKind(inject.kind);
    if (kind == FaultKind::RingCorrupt) {
        corrupt_tx_ring();
    } else if (kind == FaultKind::Flood) {
        _flooding = true;
        _flood_until_ms = _platform.millis() + FLOOD_MS;
    } else if (lua_thread_fault(kind)) {
        _link->fault_request.store(uint8_t(kind));   // taken by the Lua thread between scripts
    } else {
        _platform.inject_fault(kind);
    }
}

void M4Service::forward_call()
{
    // the request was written by the Lua thread (unprivileged on the M4 image): check it
    // before it reaches the M7, as the M7 checks it again
    const uint16_t len = _link->request_len;
    MsgHeader header;
    if (len < sizeof(CallHeader) || len > MAX_FRAME) {
        complete(-EINVAL);
        return;
    }
    memcpy(&header, _link->request, sizeof(header));
    const MsgType type = MsgType(header.type);
    if ((type != MsgType::Call && type != MsgType::Notify) || header.length != len) {
        complete(-EINVAL);
        return;
    }
    header.seq = ++_seq;
    memcpy(_link->request, &header, sizeof(header));
    const uint32_t sent_cycles = _platform.cycles();
    if (_channel->send(_link->request, len) != 0) {
        return;   // ring full: the M7 is behind, try again on the next step
    }
    if (type == MsgType::Notify) {
        complete(0);
        return;
    }
    CallHeader call;
    memcpy(&call, _link->request, sizeof(call));
    _call_seq = header.seq;
    _call_method = call.method;
    _call_sent_cycles = sent_cycles;
    _link->state.store(LuaLink::Sent);
}

void M4Service::complete(int32_t result)
{
    _call_timed = false;
    _link->result = result;
    _link->state.store(LuaLink::Done);
    _platform.wake_lua();
}

M4Service::MethodStats *M4Service::stats_for(uint16_t method)
{
    MethodStats *free_slot = nullptr;
    for (MethodStats &slot : _stats) {
        if (slot.method == method) {
            return &slot;
        }
        if (slot.method == 0 && free_slot == nullptr) {
            free_slot = &slot;
        }
    }
    if (free_slot != nullptr) {
        free_slot->method = method;
    }
    return free_slot;   // nullptr: every slot is busy this window, the call is not timed
}

StatsRow M4Service::row_of(const MethodStats &slot)
{
    StatsRow row;
    row.method = slot.method;
    row.count = slot.round_trips.count();
    row.p50_us = slot.round_trips.percentile(50);
    row.p99_us = slot.round_trips.percentile(99);
    row.max_us = slot.round_trips.max_us();
    row.total_us = slot.total_us;
    row.m7_max_us = slot.m7_max_us;
    row.lost = slot.lost;
    return row;
}

void M4Service::clear(MethodStats &slot)
{
    slot.round_trips.reset();
    slot.total_us = 0;
    slot.m7_max_us = 0;
    slot.lost = 0;
}

bool M4Service::send_row(const StatsRow &row)
{
    StatsMsg stats;
    memset(&stats, 0, sizeof(stats));
    fill_header(stats.header, MsgType::Stats, sizeof(stats), ++_seq);
    stats.method = row.method;
    stats.count = row.count;
    stats.p50_us = row.p50_us;
    stats.p99_us = row.p99_us;
    stats.max_us = row.max_us;
    stats.total_us = row.total_us;
    stats.m7_max_us = row.m7_max_us;
    stats.lost = row.lost;
    binding_method_label(row.method, stats.name, sizeof(stats.name));
    return _channel->send(&stats, sizeof(stats)) == 0;
}

void M4Service::send_stats(uint32_t now_ms)
{
    // the slots with calls this window, most round-trip time first (32 at most)
    uint32_t totals[STATS_SLOTS];
    uint8_t slot_of[STATS_SLOTS];
    uint8_t active = 0;
    for (uint8_t i = 0; i < STATS_SLOTS; i++) {
        MethodStats &slot = _stats[i];
        if (slot.method == 0) {
            continue;
        }
        if (slot.round_trips.count() == 0 && slot.lost == 0) {
            slot.method = 0;   // idle for a whole window: free the slot
            continue;
        }
        totals[active] = slot.total_us;
        slot_of[active++] = i;
    }
    uint8_t order[STATS_SLOTS];
    stats_rank(totals, active, order);
    uint8_t sent = 0;
    for (; sent < active && sent < STATS_REPORTED; sent++) {
        MethodStats &slot = _stats[slot_of[order[sent]]];
        if (!send_row(row_of(slot))) {
            return;   // ring full: this and the remaining rows wait for the next step
        }
        clear(slot);
    }
    if (sent < active || _untimed.count != 0 || _untimed.lost != 0) {
        // the folded methods keep their totals, so the rows add up to the window's calls; the
        // runtime's own system methods fold into their own row: m4:other holds binding calls only
        StatsRow other = _untimed;
        other.method = uint16_t(SystemMethod::Other);
        StatsRow system_other {};
        system_other.method = uint16_t(SystemMethod::SystemOther);
        for (uint8_t i = sent; i < active; i++) {
            const StatsRow row = row_of(_stats[slot_of[order[i]]]);
            stats_add(row.method < FIRST_BINDING_METHOD ? system_other : other, row);
        }
        // each row is cleared once sent, so a full ring repeats neither
        if (other.count != 0 || other.lost != 0) {
            if (!send_row(other)) {
                return;
            }
            for (uint8_t i = sent; i < active; i++) {
                MethodStats &slot = _stats[slot_of[order[i]]];
                if (slot.method >= FIRST_BINDING_METHOD) {
                    clear(slot);
                }
            }
            memset(&_untimed, 0, sizeof(_untimed));
        }
        if (system_other.count != 0 || system_other.lost != 0) {
            if (!send_row(system_other)) {
                return;
            }
            for (uint8_t i = sent; i < active; i++) {
                MethodStats &slot = _stats[slot_of[order[i]]];
                if (slot.method < FIRST_BINDING_METHOD) {
                    clear(slot);
                }
            }
        }
    }
    _last_stats_ms = now_ms;
}

// flood() passes that found the ring full: tests read it to show a flood filled the ring
static volatile uint32_t flood_ring_full;

void M4Service::flood()
{
    // valid Notify frames until the ring is full: the M7 must bound the time it spends on them
    CallHeader noop;
    fill_header(noop.header, MsgType::Notify, sizeof(noop), 0);
    noop.method = uint16_t(SystemMethod::Noop);
    noop.argc = 0;
    noop.reserved = 0;
    // the service may sleep 5 ms between steps, so each step fills the ring to the top; bounded
    // by twice the frames one ring (half the data) holds, as the M7 drains it meanwhile
    const uint32_t max_frames = (CHANNEL_WINDOW_SIZE - Channel::kOverhead) / sizeof(noop);
    for (uint32_t i = 0; i < max_frames; i++) {
        noop.header.seq = ++_seq;
        if (_channel->send(&noop, sizeof(noop)) != 0) {
            flood_ring_full = flood_ring_full + 1;
            return;
        }
    }
}

void M4Service::corrupt_tx_ring()
{
    // an unaligned producer index the M7's ring reader rejects (SpscRing::desynced)
    auto *control = reinterpret_cast<Channel::Control *>(reinterpret_cast<uintptr_t>(_window) + PeerHeader::kSize);
    control->m4ToM7.head.value.store(0x7FFFFFF1U);
}

}  // namespace ScriptingM4
