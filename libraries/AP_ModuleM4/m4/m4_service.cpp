#include "m4_service.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <CubePilotFW/PeerHeader.h>
#include <AP_ModuleM4_Layout.h>

namespace ModuleM4 {

using CubePilotFW::Channel;
namespace PeerHeader = CubePilotFW::PeerHeader;

void M4Service::init(Channel &channel, uint8_t *window, ModuleLink &link, const M4Platform &platform)
{
    _channel = &channel;
    _window = window;
    _link = &link;
    _platform = platform;
    if (_platform.cycles_per_us == 0) {
        _platform.cycles_per_us = 1;
    }
    _seq = 0;
    _protocol_errors = 0;
    _ready = false;
    _rx_stalled = false;
    _call_timed = false;
    _flooding = false;
    compute_crc_self_check();
    // a CRC that disagrees with zlib would fail every ring frame: fault before the first beat,
    // so the M7 reports it as a start fault rather than this session running on bad CRCs
    if (!_crc_self_check.passed()) {
        _platform.inject_fault(FaultKind::HardFault);
    }
}

void M4Service::compute_crc_self_check()
{
    Crc32 crc;
    crc32_begin(crc);
    crc32_feed(crc, reinterpret_cast<const uint8_t *>("123456789"), 9);
    _crc_self_check.short_check = crc32_finish(crc);
    uint8_t pattern[256];
    crc32_begin(crc);
    for (uint32_t offset = 0; offset < 2048U; offset += uint32_t(sizeof(pattern))) {
        for (uint32_t i = 0; i < sizeof(pattern); i++) {
            pattern[i] = uint8_t((offset + i) * 7U + 3U);
        }
        // one chunk in two pieces: a continued feed and the tail-byte path are both exercised
        const uint32_t odd = offset == 768U ? 233U : uint32_t(sizeof(pattern));
        crc32_feed(crc, pattern, odd);
        if (odd != sizeof(pattern)) {
            crc32_feed(crc, pattern + odd, uint32_t(sizeof(pattern)) - odd);
        }
    }
    _crc_self_check.pattern_check = crc32_finish(crc);
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
    if (_ready && _link->state.load() == ModuleLink::Posted) {
        if (!_call_timed) {
            _call_timed = true;
            _call_posted_ms = _platform.millis();
        }
        forward_call();
    }
    const bool sent = _link->state.load() != ModuleLink::Posted;
    _platform.unlock();
    return sent;
}

void M4Service::step_locked()
{
    drain();
    const uint32_t now_ms = _platform.millis();
    const uint8_t state = _link->state.load();
    if (state == ModuleLink::Posted && !_call_timed) {
        // the call timeout runs from posting: waiting for Ready or ring space counts too
        _call_timed = true;
        _call_posted_ms = now_ms;
    }
    if ((state == ModuleLink::Posted || state == ModuleLink::Sent) && now_ms - _call_posted_ms > CALL_TIMEOUT_MS) {
        complete(-ETIMEDOUT);
    } else if (_ready && state == ModuleLink::Posted) {
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
        _protocol_errors++;
        return;
    }
    switch (MsgType(header.type)) {
    case MsgType::Ready:
        _ready = true;
        return;
    case MsgType::FaultInject:
        handle_fault_inject(buf);
        return;
    case MsgType::Control:
        return;   // not acted on by this service yet
    case MsgType::BindingReply:
    case MsgType::SystemReply:
        if (_link->state.load() == ModuleLink::Sent && header.seq == _call_seq) {
            memcpy(_link->reply, buf, len);
            complete(len);
            return;
        }
        break;
    case MsgType::Stats:
    case MsgType::BindingCall:
    case MsgType::SystemCall:
        break;   // M4 -> M7 only
    }
    _protocol_errors++;
}

void M4Service::handle_fault_inject(const uint8_t *buf)
{
    FaultInjectMsg inject;
    memcpy(&inject, buf, sizeof(inject));
    if (inject.kind > FAULT_KIND_MAX) {
        return;   // before FaultKind's 8 bits truncate it
    }
    switch (FaultKind(inject.kind)) {
    case FaultKind::RingCorrupt:
        corrupt_tx_ring();
        return;
    case FaultKind::Flood:
        _flooding = true;
        _flood_until_ms = _platform.millis() + FLOOD_MS;
        return;
    case FaultKind::HardFault:
    case FaultKind::MemManage:
    case FaultKind::Stall:
    case FaultKind::Lockup:
    case FaultKind::HandlerFault:
        _platform.inject_fault(FaultKind(inject.kind));
        return;
    case FaultKind::None:
        break;
    }
}

void M4Service::forward_call()
{
    // the request was written by the module thread (unprivileged on the M4 image): check it
    // before it reaches the M7, as the M7 checks it again
    const uint16_t len = _link->request_len;
    MsgHeader header;
    if (len > MAX_FRAME || !parse_header(_link->request, len, header) ||
        (MsgType(header.type) != MsgType::BindingCall && MsgType(header.type) != MsgType::SystemCall)) {
        complete(-EINVAL);   // only calls go M4 -> M7 from the module thread
        return;
    }
    header.seq = ++_seq;
    memcpy(_link->request, &header, sizeof(header));
    if (_channel->send(_link->request, len) != 0) {
        return;   // ring full: the M7 is behind, try again on the next step
    }
    _call_seq = header.seq;
    _link->state.store(ModuleLink::Sent);
}

void M4Service::complete(int32_t result)
{
    _call_timed = false;
    _link->result = result;
    _link->state.store(ModuleLink::Done);
    if (_platform.wake_module != nullptr) {
        _platform.wake_module();
    }
}

static volatile uint32_t flood_ring_full;

void M4Service::flood()
{
    // valid notify frames until the ring is full: the M7 must bound the time it spends on them
    SystemCallHeader noop;
    fill_header(noop.header, MsgType::SystemCall, sizeof(noop), 0);
    noop.method = uint16_t(SystemMethod::Noop);
    noop.flags = CALL_FLAG_NOTIFY;
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

bool M4Service::send_row(const StatsRow &row, const char *name)
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
    for (size_t i = 0; i + 1 < sizeof(stats.name) && name[i] != '\0'; i++) {
        stats.name[i] = name[i];   // terminated by the memset
    }
    return _channel->send(&stats, sizeof(stats)) == 0;
}

void M4Service::corrupt_tx_ring()
{
    // an unaligned producer index the M7's ring reader rejects (SpscRing::desynced)
    auto *control = reinterpret_cast<Channel::Control *>(reinterpret_cast<uintptr_t>(_window) + PeerHeader::kSize);
    control->m4ToM7.head.value.store(0x7FFFFFF1U);
}

}  // namespace ModuleM4
