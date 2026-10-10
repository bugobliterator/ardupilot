#include "m4_service.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <CubePilotFW/PeerHeader.h>
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
    _ready = false;
    _rx_stalled = false;
    _flooding = false;
    _last_stats_ms = platform.millis();
}

void M4Service::step()
{
    drain();
    const uint32_t now_ms = _platform.millis();
    if (_link->state.load() == LuaLink::Sent && now_ms - _call_sent_ms > CALL_TIMEOUT_MS) {
        MethodStats *stats = stats_for(_call_method);
        if (stats != nullptr) {
            stats->lost++;
        }
        complete(-ETIMEDOUT);
    }
    // calls wait for the M7's Ready: frames sent before its re-handshake would be dropped
    if (_ready && _link->state.load() == LuaLink::Posted) {
        forward_call(now_ms);
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

void M4Service::forward_call(uint32_t now_ms)
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
    _call_sent_ms = now_ms;
    _link->state.store(LuaLink::Sent);
}

void M4Service::complete(int32_t result)
{
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

void M4Service::send_stats(uint32_t now_ms)
{
    for (MethodStats &slot : _stats) {
        if (slot.method == 0) {
            continue;
        }
        if (slot.round_trips.count() == 0 && slot.lost == 0) {
            slot.method = 0;   // idle for a whole window: free the slot
            continue;
        }
        StatsMsg stats;
        fill_header(stats.header, MsgType::Stats, sizeof(stats), ++_seq);
        stats.method = slot.method;
        stats.reserved = 0;
        stats.count = slot.round_trips.count();
        stats.p50_us = slot.round_trips.percentile(50);
        stats.p99_us = slot.round_trips.percentile(99);
        stats.max_us = slot.round_trips.max_us();
        stats.total_us = slot.total_us;
        stats.m7_max_us = slot.m7_max_us;
        stats.lost = slot.lost;
        if (_channel->send(&stats, sizeof(stats)) != 0) {
            return;   // ring full: keep this and the remaining windows, retry on the next step
        }
        slot.round_trips.reset();
        slot.total_us = 0;
        slot.m7_max_us = 0;
        slot.lost = 0;
    }
    _last_stats_ms = now_ms;
}

void M4Service::flood()
{
    // valid Notify frames until the ring is full: the M7 must bound the time it spends on them
    CallHeader noop;
    fill_header(noop.header, MsgType::Notify, sizeof(noop), 0);
    noop.method = uint16_t(SystemMethod::Noop);
    noop.argc = 0;
    noop.reserved = 0;
    for (uint16_t i = 0; i < 256; i++) {
        noop.header.seq = ++_seq;
        if (_channel->send(&noop, sizeof(noop)) != 0) {
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
