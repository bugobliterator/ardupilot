#include "m4_service.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <CubePilotFW/PeerHeader.h>
#include <AP_ScriptingM4_Layout.h>

namespace ScriptingM4 {

using CubePilotFW::Channel;
namespace PeerHeader = CubePilotFW::PeerHeader;

void M4Service::init(Channel &channel, uint8_t *window, const M4Platform &platform)
{
    _channel = &channel;
    _window = window;
    _platform = platform;
    if (_platform.cycles_per_us == 0) {
        _platform.cycles_per_us = 1;
    }
    _rx_stalled = false;
    _round_trips.reset();
    _round_trip_total_us = 0;
    _m7_max_us = 0;
    _lost = 0;
    _seq = 0;
    _ping_seq = 0;
    _ping_outstanding = false;
    _last_ping_ms = platform.millis();
    _last_stats_ms = _last_ping_ms;
}

void M4Service::step()
{
    drain();
    const uint32_t now_ms = _platform.millis();
    if (_ping_outstanding && now_ms - _ping_sent_ms > PING_TIMEOUT_MS) {
        _ping_outstanding = false;
        _lost++;
    }
    if (!_ping_outstanding && now_ms - _last_ping_ms >= PING_INTERVAL_MS) {
        send_ping(now_ms);
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
        uint8_t buf[MAX_MESSAGE];
        uint16_t len = 0;
        const int ret = _channel->receive(buf, sizeof(buf), &len);
        if (ret != 0) {
            if (ret != -EAGAIN) {
                _rx_stalled = true;   // e.g. -EMSGSIZE or -EBADMSG: the frame is not consumed
            }
            return;
        }
        handle(buf, len);
    }
}

void M4Service::handle(const uint8_t *buf, uint16_t len)
{
    MsgHeader header;
    if (!parse_header(buf, len, header)) {
        return;
    }
    switch (MsgType(header.type)) {
    case MsgType::Pong: {
        if (!_ping_outstanding || header.seq != _ping_seq) {
            return;   // reply to a ping already counted as lost
        }
        PongMsg pong;
        memcpy(&pong, buf, sizeof(pong));
        const uint32_t round_trip_us = (_platform.cycles() - _ping_sent_cycles) / _platform.cycles_per_us;
        _round_trips.add(round_trip_us);
        _round_trip_total_us += round_trip_us;
        const uint32_t m7_us = pong.m7_wake_us + pong.m7_exec_us;
        if (m7_us > _m7_max_us) {
            _m7_max_us = m7_us;
        }
        _ping_outstanding = false;
        return;
    }
    case MsgType::FaultInject: {
        FaultInjectMsg inject;
        memcpy(&inject, buf, sizeof(inject));
        if (inject.kind == uint32_t(FaultKind::RingCorrupt)) {
            corrupt_tx_ring();
        } else if (inject.kind != uint32_t(FaultKind::None) && inject.kind <= FAULT_KIND_MAX) {
            _platform.inject_fault(FaultKind(inject.kind));
        }
        return;
    }
    case MsgType::Ping:
    case MsgType::Stats:
    case MsgType::Call:
    case MsgType::Reply:
    case MsgType::Notify:
    case MsgType::Ready:
        return;   // M4 -> M7 only, or not served before the Call Path
    }
}

void M4Service::send_ping(uint32_t now_ms)
{
    PingMsg ping;
    fill_header(ping.header, MsgType::Ping, sizeof(ping), ++_seq);
    _last_ping_ms = now_ms;
    const uint32_t sent_cycles = _platform.cycles();
    if (_channel->send(&ping, sizeof(ping)) != 0) {
        return;   // ring full: the M7 is behind, try again next interval
    }
    _ping_outstanding = true;
    _ping_seq = _seq;
    _ping_sent_ms = now_ms;
    _ping_sent_cycles = sent_cycles;
}

void M4Service::send_stats(uint32_t now_ms)
{
    StatsMsg stats;
    fill_header(stats.header, MsgType::Stats, sizeof(stats), ++_seq);
    stats.count = _round_trips.count();
    stats.p50_us = _round_trips.percentile(50);
    stats.p99_us = _round_trips.percentile(99);
    stats.max_us = _round_trips.max_us();
    stats.total_us = _round_trip_total_us;
    stats.m7_max_us = _m7_max_us;
    stats.lost = _lost;
    if (_channel->send(&stats, sizeof(stats)) != 0) {
        return;   // keep the window and retry on the next step
    }
    _last_stats_ms = now_ms;
    _round_trips.reset();
    _round_trip_total_us = 0;
    _m7_max_us = 0;
    _lost = 0;
}

void M4Service::corrupt_tx_ring()
{
    // an unaligned producer index the M7's ring reader rejects (SpscRing::desynced)
    auto *control = reinterpret_cast<Channel::Control *>(reinterpret_cast<uintptr_t>(_window) + PeerHeader::kSize);
    control->m4ToM7.head.value.store(0x7FFFFFF1U);
}

}  // namespace ScriptingM4
