#include "m4_service.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <CubePilotFW/PeerHeader.h>
#include <AP_RustM4_Layout.h>

namespace RustM4 {

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
    // a new session: no names, statistics or M7 session yet
    memset(_stats, 0, sizeof(_stats));
    memset(&_untimed, 0, sizeof(_untimed));
    _name_count = 0;
    _session = 0;
    _have_session = false;
    _lost = 0;
    _last_stats_ms = _platform.millis();
    _seq = 0;
    _protocol_errors = 0;
    _ready = false;
    _rx_stalled = false;
    _call_timed = false;
    _flooding = false;
    _config_pending = false;
    _config_ready = false;
    _config_seq = 0;
    _mirror.clear();
    _mirror_hits = 0;
    _mirror_misses = 0;
    _post_drops = 0;
    _reported_mirror_hits = 0;
    _reported_mirror_misses = 0;
    _reported_post_drops = 0;
    _ended_modules = 0;
    _call_probed = false;
    _lock_timed = false;
    _lock_cycles = 0;
    _receive_cycles = 0;
    _received_cycles = 0;
    _service_wake = WakeNote {};
    _module_wake = WakeNote {};
    _module_draining = false;
    _deferred_fault = uint8_t(FaultKind::None);
    M4CallProbe *probe = _platform.probe;
    if (probe != nullptr) {
        probe->enabled.store(false);   // until this session's Config asks for it
        probe->last.seq = 0;
        if (probe->clock_us != nullptr) {
            // the cost of one stamp, reported in every trailer
            static const uint32_t READS = 16;
            const uint32_t start = _platform.cycles();
            for (uint32_t i = 0; i < READS; i++) {
                (void)probe->clock_us();
            }
            probe->stamp_cycles = (_platform.cycles() - start) / READS;
        }
    }
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

bool M4Service::peer_asleep() const
{
    // Channel::init() checked the window's 32 B alignment
    const void *shared = _channel->header() + PeerHeader::kSize;
    const auto *control = static_cast<const Channel::Control *>(shared);
    return control->asleep[uint8_t(PeerHeader::Core::M7)].load() == 1;
}

uint32_t M4Service::probe_cycles() const
{
    const M4CallProbe *probe = _platform.probe;
    if (probe == nullptr || probe->cycles == nullptr || !probe->enabled.load(std::memory_order_relaxed)) {
        return 0;
    }
    return probe->cycles();
}

void M4Service::lock_timed()
{
    const uint32_t start = probe_cycles();
    _platform.lock();
    _lock_timed = start != 0;
    _lock_cycles = _lock_timed ? cycles_between(start, probe_cycles()) : 0;
}

void M4Service::drain_step()
{
    // a reply is drained under the lock forward_posted() holds from send to Sent: it is never
    // seen before its call is Sent and dropped as stale
    lock_timed();
    if (_deferred_fault != uint8_t(FaultKind::None)) {
        const FaultKind kind = FaultKind(_deferred_fault);
        _deferred_fault = uint8_t(FaultKind::None);
        _platform.inject_fault(kind);
    }
    drain();
    if (_ready && _link->state.load() == ModuleLink::Posted) {
        // calls wait for the M7's Ready: frames sent before its re-handshake would be dropped
        time_posted_call();
        forward_call(true);
    }
    _platform.unlock();
}

bool M4Service::drain_step_for_module(bool poll_timeout, bool stamped, uint32_t isr_us, uint32_t wake_us,
                                      uint16_t isr_to_thread_cycles)
{
    lock_timed();
    _module_wake = WakeNote { poll_timeout, stamped, isr_us, wake_us, isr_to_thread_cycles };
    _module_draining = true;
    drain();
    if (_ready && _link->state.load() == ModuleLink::Posted) {
        time_posted_call();
        forward_call(true);
    }
    _module_draining = false;
    const bool deferred = _deferred_fault != uint8_t(FaultKind::None);
    _platform.unlock();
    return deferred;
}

bool M4Service::forward_posted()
{
    lock_timed();
    if (_ready && _link->state.load() == ModuleLink::Posted) {
        time_posted_call();
        forward_call(false);
    }
    const bool sent = _link->state.load() != ModuleLink::Posted;
    // the probed call this thread just sent: its reply cannot be drained before the unlock
    M4CallProbe *probe = sent && _call_probed && _link->state.load() == ModuleLink::Sent ? _platform.probe : nullptr;
    const uint32_t unlock_cycles = probe != nullptr ? probe_cycles_now(*probe) : 0;
    _platform.unlock();
    if (probe != nullptr) {
        probe_cycle_part(probe->current, M4CyclePart::FwdUnlock, unlock_cycles, probe_cycles_now(*probe));
    }
    return sent;
}

void M4Service::time_posted_call()
{
    if (!_call_timed) {
        _call_timed = true;
        _call_posted_ms = _platform.millis();
        _call_posted_cycles = _platform.cycles();
    }
}

void M4Service::housekeeping_step()
{
    _platform.lock();
    _lock_timed = false;
    const uint32_t now_ms = _platform.millis();
    const uint8_t state = _link->state.load();
    if (state == ModuleLink::Posted && !_call_timed) {
        _call_timed = true;
        _call_posted_ms = now_ms;
        _call_posted_cycles = _platform.cycles();
    }
    if ((state == ModuleLink::Posted || state == ModuleLink::Sent) && now_ms - _call_posted_ms > CALL_TIMEOUT_MS) {
        if (state == ModuleLink::Sent) {
            count_lost();   // its reply, if it still comes, is dropped by seq
        } else if (posted_binding_on_link()) {
            _post_drops++;   // the module got OK; its frame never reached the ring
        }
        complete(-ETIMEDOUT);
    } else if (_ready && state == ModuleLink::Posted) {
        forward_call(true);   // the ring was full at the last try
    }
    // sent again if its reply did not come (the M7's ring was full)
    if (_ready && _config_pending && (_config_seq == 0 || now_ms - _config_sent_ms > CALL_TIMEOUT_MS)) {
        send_config(now_ms);
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
    _platform.unlock();
}

void M4Service::drain()
{
    // bounded, so a flooding M7 cannot keep the M4 from beating
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t len = 0;
        // a probed call is Sent: its reply may be the next frame
        const uint32_t receive_cycles = _call_probed ? probe_cycles() : 0;
        const int ret = _channel->receive(_rx, sizeof(_rx), &len);
        if (receive_cycles != 0) {
            _received_cycles = probe_cycles();
            _receive_cycles = cycles_between(receive_cycles, _received_cycles);
        } else {
            _received_cycles = 0;
        }
        if (ret != 0) {
            if (ret != -EAGAIN) {
                _rx_stalled = true;   // e.g. -EMSGSIZE or -EBADMSG: the frame is not consumed
            }
            return;
        }
        handle(_rx, len);
        // the module thread's call is answered: frames after its reply are left to the service,
        // which the module thread's prepare_module_sleep() then wakes
        if (_module_draining && _link->state.load() == ModuleLink::Done) {
            return;
        }
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
        handle_ready(header);
        return;
    case MsgType::FaultInject:
        handle_fault_inject(buf);
        return;
    case MsgType::Control:
        handle_control(buf, header);
        return;
    case MsgType::BindingReply:
    case MsgType::SystemReply:
        handle_reply(buf, len, header);
        return;
    case MsgType::ParameterChanged:
        handle_parameter_changed(buf, len, header);
        return;
    case MsgType::Stats:
    case MsgType::Counters:
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
    if (inject.kind == INJECT_MODULE_SELFTEST) {
        // not a fault: the built-in check of the module thread path
        if (_platform.start_selftest != nullptr) {
            _platform.start_selftest();
        }
        return;
    }
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
        if (_module_draining) {
            // a stall must stop the service thread, not the module thread below it
            _deferred_fault = uint8_t(inject.kind);
            return;
        }
        _platform.inject_fault(FaultKind(inject.kind));
        return;
    case FaultKind::None:
        break;
    }
}

void M4Service::forward_call(bool from_step)
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
    // BindingCallHeader and SystemCallHeader both carry their flags at this offset
    static_assert(offsetof(BindingCallHeader, flags) == offsetof(SystemCallHeader, flags), "flags offset");
    uint16_t flags;
    memcpy(&flags, _link->request + offsetof(SystemCallHeader, flags), sizeof(flags));
    uint16_t key;
    uint8_t reply_type;
    M4CallProbe *probe = nullptr;
    if (MsgType(header.type) == MsgType::BindingCall) {
        BindingCallHeader call;
        memcpy(&call, _link->request, sizeof(call));
        key = call.id < KEY_OTHER ? uint16_t(call.id) : KEY_OTHER;
        reply_type = uint8_t(MsgType::BindingReply);
        if ((flags & CALL_FLAG_PROBE) != 0 && probing(_platform.probe) && len >= sizeof(CallProbeTrailer) &&
            len == sizeof(call) + call.in_size + sizeof(CallProbeTrailer)) {
            probe = _platform.probe;
        }
    } else {
        SystemCallHeader call;
        memcpy(&call, _link->request, sizeof(call));
        key = uint16_t(SYSTEM_KEY | call.method);
        reply_type = uint8_t(MsgType::SystemReply);
        forget_ended_module();
        CallProbeTrailer trailer;
        if (probing(_platform.probe) && ping_call_trailer(_link->request, len, call, trailer)) {
            probe = _platform.probe;
        }
    }
    header.seq = ++_seq;
    memcpy(_link->request, &header, sizeof(header));
    if (probe != nullptr) {
        const uint32_t pre_send_us = probe->clock_us();
        memcpy(_link->request + len - sizeof(CallProbeTrailer) + offsetof(CallProbeTrailer, pre_send_us),
               &pre_send_us, sizeof(pre_send_us));
    }
    const uint32_t sent_cycles = _platform.cycles();
    // with the M7 asleep, send() rings its doorbell too
    const bool bell = probe != nullptr && peer_asleep();
    const uint32_t send_cycles = probe != nullptr ? probe_cycles_now(*probe) : 0;
    if (_channel->send(_link->request, len) != 0) {
        return;   // ring full: the M7 is behind, try again on the next step
    }
    if (probe != nullptr) {
        probe_cycle_part(probe->current, bell ? M4CyclePart::FwdBell : M4CyclePart::FwdRing, send_cycles,
                         probe_cycles_now(*probe));
        if (_lock_timed) {
            probe_cycle_part(probe->current, M4CyclePart::FwdLock, 0, _lock_cycles);
        }
    }
    if ((flags & CALL_FLAG_NOTIFY) != 0) {
        if (MsgType(header.type) == MsgType::BindingCall) {
            // a posted binding: its round trip is the time from posting to the ring copy
            record_posted(key, (sent_cycles - _call_posted_cycles) / _platform.cycles_per_us);
        }
        complete(0);   // no reply
        return;
    }
    _call_probed = probe != nullptr;
    if (probe != nullptr) {
        probe->current.sent_us = probe->clock_us();
        probe->current.seq = header.seq;
        if (from_step) {
            probe->current.flags |= PROBE_M4_SENT_BY_STEP;
        }
    }
    _call_seq = header.seq;
    _call_session = _session;
    _call_key = key;
    _call_reply_type = reply_type;
    _call_sent_cycles = sent_cycles;
    _link->state.store(ModuleLink::Sent);
}

// the M7 asks a module to stop; seq carries its session, so a stop of an older one is dropped
void M4Service::handle_control(const uint8_t *buf, const MsgHeader &header)
{
    ControlMsg control;
    memcpy(&control, buf, sizeof(control));   // parse_header() checked the length
    if (!_have_session || header.seq != _session || control.action != CONTROL_STOP) {
        _protocol_errors++;
        return;
    }
    if (_platform.stop_module != nullptr) {
        _platform.stop_module(control.module);
    }
}

void M4Service::handle_ready(const MsgHeader &header)
{
    // the M7 repeats Ready with its session in seq; a new session cannot answer this one's call
    if (_have_session && header.seq != _session && _link->state.load() == ModuleLink::Sent) {
        count_lost();
        complete(-ECONNRESET);
    }
    if (!_have_session || header.seq != _session) {
        // one Config per session
        _config_pending = true;
        _config_ready = false;
        _config_seq = 0;
        // a new M7 session has no handles
        _mirror.clear();
        _ended_modules = 0;
    }
    _session = header.seq;
    _have_session = true;
    _ready = true;
}

void M4Service::send_config(uint32_t now_ms)
{
    ConfigRequest request;
    memset(&request, 0, sizeof(request));
    fill_header(request.call.header, MsgType::SystemCall, sizeof(request), ++_seq);
    request.call.method = uint16_t(SystemMethod::Config);
    request.crc_short = _crc_self_check.short_check;
    request.crc_pattern = _crc_self_check.pattern_check;
    if (_channel->send(&request, sizeof(request)) == 0) {
        _config_seq = request.call.header.seq;
        _config_sent_ms = now_ms;
    }
}

// a Config reply the M7 refused or malformed leaves config_ready() false for the session
void M4Service::handle_config_reply(const uint8_t *buf, uint16_t len)
{
    _config_pending = false;
    ConfigReply reply;
    if (len != sizeof(reply)) {
        return;
    }
    memcpy(&reply, buf, sizeof(reply));
    if (reply.reply.method != uint16_t(SystemMethod::Config) || reply.reply.transport != uint8_t(ReplyStatus::Ok)) {
        return;
    }
    _config_reply = reply;
    _config_ready = true;
    if (_platform.probe != nullptr && _platform.probe->clock_us != nullptr) {
        _platform.probe->enabled.store((reply.m4_options & M4_OPTION_CALL_PROBE) != 0);
    }
    if (_platform.m7_millis != nullptr) {
        _platform.m7_millis(reply.m7_millis);
    }
}

void M4Service::handle_reply(const uint8_t *buf, uint16_t len, const MsgHeader &header)
{
    if (MsgType(header.type) == MsgType::SystemReply && _config_pending && _config_seq != 0 &&
        header.seq == _config_seq) {
        handle_config_reply(buf, len);
        return;
    }
    // seq is never reused while this M4 runs: a reply to a call that timed out or lost its
    // session matches no Sent call
    if (_link->state.load() != ModuleLink::Sent || header.seq != _call_seq || _call_session != _session ||
        header.type != _call_reply_type) {
        _protocol_errors++;
        return;
    }
    WakeNote &wake = _module_draining ? _module_wake : _service_wake;
    if (_call_probed) {
        M4CallProbe &probe = *_platform.probe;
        if (_lock_timed) {
            probe_cycle_part(probe.current, M4CyclePart::DrainLock, 0, _lock_cycles);
        }
        if (_received_cycles != 0) {
            probe_cycle_part(probe.current, M4CyclePart::DrainRing, 0, _receive_cycles);
        }
        if (wake.isr_to_thread_cycles != 0) {
            probe_cycle_part(probe.current, M4CyclePart::IsrToThread, 0, wake.isr_to_thread_cycles);
        }
        probe.current.reply_seen_us = probe.clock_us();
        if (wake.poll_timeout) {
            probe.current.flags |= PROBE_M4_POLL_WAKE;
        }
        if (wake.stamped) {
            probe.current.reply_isr_us = wake.isr_us;
            probe.current.reply_wake_us = wake.wake_us;
            probe.current.stamps |= PROBE_STAMP_REPLY_WAKE;
        }
    }
    wake.stamped = false;   // one wait's stamps for one reply
    wake.isr_to_thread_cycles = 0;
    const uint32_t round_trip_us = (_platform.cycles() - _call_sent_cycles) / _platform.cycles_per_us;
    uint32_t m7_us = 0;
    if (MsgType(header.type) == MsgType::BindingReply) {
        BindingReplyHeader reply;
        memcpy(&reply, buf, sizeof(reply));
        m7_us = reply.m7_wake_us + reply.m7_exec_us;
        BindingCallHeader call;
        memcpy(&call, _link->request, sizeof(call));
        if ((reply.reserved2 & REPLY_FLAG_MIRROR_RESYNC) != 0 && call.module < MAX_MODULES) {
            _mirror.mark_module_stale(uint8_t(call.module));   // a ParameterChanged did not reach us
        }
    }
    MethodStats *stats = stats_for(_call_key);
    if (stats != nullptr) {
        stats->round_trips.add(round_trip_us);
        stats->total_us += round_trip_us;
        if (m7_us > stats->m7_max_us) {
            stats->m7_max_us = m7_us;
        }
    } else {
        _untimed.count++;
        _untimed.total_us += round_trip_us;
        // no histogram: the percentiles of these calls are bounded by their largest
        if (round_trip_us > _untimed.max_us) {
            _untimed.p50_us = _untimed.p99_us = _untimed.max_us = round_trip_us;
        }
        if (m7_us > _untimed.m7_max_us) {
            _untimed.m7_max_us = m7_us;
        }
    }
    const uint32_t copy_cycles = _call_probed && _received_cycles != 0 ? probe_cycles_now(*_platform.probe) : 0;
    memcpy(_link->reply, buf, len);
    if (copy_cycles != 0) {
        M4CallStamps &current = _platform.probe->current;
        probe_cycle_part(current, M4CyclePart::DrainParse, _received_cycles, copy_cycles);
        current.drain_copied_cycles = probe_cycles_now(*_platform.probe);
        probe_cycle_part(current, M4CyclePart::DrainCopy, copy_cycles, current.drain_copied_cycles);
    }
    complete(len);
}

void M4Service::count_lost()
{
    _lost++;
    MethodStats *stats = stats_for(_call_key);
    if (stats != nullptr) {
        stats->lost++;
    } else {
        _untimed.lost++;
    }
}

bool M4Service::posted_binding_on_link() const
{
    BindingCallHeader call;
    return parse_binding_call(_link->request, _link->request_len, call) && (call.flags & CALL_FLAG_NOTIFY) != 0;
}

void M4Service::record_posted(uint16_t key, uint32_t post_us)
{
    MethodStats *stats = stats_for(key);
    if (stats == nullptr) {
        _untimed.count++;
        _untimed.total_us += post_us;
        return;
    }
    stats->posted = true;
    stats->round_trips.add(post_us);
    stats->total_us += post_us;
}

void M4Service::complete(int32_t result)
{
    _call_timed = false;
    _link->result = result;
    _link->state.store(ModuleLink::Done);
    if (_platform.wake_module != nullptr) {
        _platform.wake_module();
    }
    if (_call_probed && result > 0) {
        probe_stamp(*_platform.probe, _platform.probe->current.signalled_us, PROBE_STAMP_SIGNALLED);
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

// text, terminated and truncated to size (no strncpy: not on the privileged import list)
static uint32_t copy_text(char *name, uint32_t size, const char *text)
{
    uint32_t i = 0;
    for (; text[i] != 0 && i + 1 < size; i++) {
        name[i] = text[i];
    }
    name[i] = 0;
    return i;
}

// prefix, then value in decimal; terminated, truncated to size
static void name_with_number(char *name, uint32_t size, const char *prefix, uint32_t value)
{
    char digits[10];
    uint8_t count = 0;
    do {
        digits[count++] = char('0' + value % 10U);
        value /= 10U;
    } while (value != 0);
    uint32_t i = copy_text(name, size, prefix);
    while (count != 0 && i + 1 < size) {
        name[i++] = digits[--count];
    }
    name[i] = 0;
}

void M4Service::set_binding_name(uint32_t id, const char *name)
{
    if (_name_count == BINDING_NAMES || name == nullptr) {
        return;   // later rows are named id:<n>
    }
    _names[_name_count].id = id;
    copy_text(_names[_name_count].name, sizeof(_names[0].name), name);
    _name_count++;
}

void M4Service::row_name(uint16_t key, char *name, uint32_t size) const
{
    if (key == KEY_OTHER) {
        copy_text(name, size, "m4:other");
        return;
    }
    if (key == KEY_SYSTEM_OTHER) {
        copy_text(name, size, "m4:sysother");
        return;
    }
    if ((key & SYSTEM_KEY) != 0) {
        name_with_number(name, size, "sys:", key & ~SYSTEM_KEY);
        return;
    }
    for (uint8_t i = 0; i < _name_count; i++) {
        if (_names[i].id == key) {
            copy_text(name, size, _names[i].name);
            return;
        }
    }
    name_with_number(name, size, "id:", key);
}

M4Service::MethodStats *M4Service::stats_for(uint16_t key)
{
    MethodStats *free_slot = nullptr;
    for (MethodStats &slot : _stats) {
        if (slot.used && slot.key == key) {
            return &slot;
        }
        if (!slot.used && free_slot == nullptr) {
            free_slot = &slot;
        }
    }
    if (free_slot != nullptr) {
        free_slot->used = true;
        free_slot->posted = false;
        free_slot->key = key;
    }
    return free_slot;   // nullptr: every slot is busy this window, the call is not timed
}

StatsRow M4Service::row_of(const MethodStats &slot)
{
    StatsRow row;
    row.method = slot.key;
    row.count = slot.round_trips.count();
    row.p50_us = slot.round_trips.percentile(50);
    row.p99_us = slot.round_trips.percentile(99);
    row.max_us = slot.round_trips.max_us();
    row.total_us = slot.total_us;
    row.m7_max_us = slot.m7_max_us;
    row.lost = slot.lost;
    row.flags = slot.posted ? STATS_ROW_POSTED : 0;
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
    stats.reserved = row.flags;
    row_name(row.method, stats.name, sizeof(stats.name));
    return _channel->send(&stats, sizeof(stats)) == 0;
}

void M4Service::send_stats(uint32_t now_ms)
{
    // the slots with calls this window, most round-trip time first
    uint32_t totals[STATS_SLOTS];
    uint8_t slot_of[STATS_SLOTS];
    uint8_t active = 0;
    for (uint8_t i = 0; i < STATS_SLOTS; i++) {
        MethodStats &slot = _stats[i];
        if (!slot.used) {
            continue;
        }
        if (slot.round_trips.count() == 0 && slot.lost == 0) {
            slot.used = false;   // idle for a whole window: free the slot
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
        // the folded keys keep their totals, so the rows add up to the window's calls; system
        // methods fold into their own row: m4:other holds binding calls only
        StatsRow other = _untimed;
        other.method = KEY_OTHER;
        StatsRow system_other {};
        system_other.method = KEY_SYSTEM_OTHER;
        for (uint8_t i = sent; i < active; i++) {
            const StatsRow row = row_of(_stats[slot_of[order[i]]]);
            stats_add((row.method & SYSTEM_KEY) != 0 ? system_other : other, row);
        }
        // each row is cleared once sent, so a full ring repeats neither
        if (other.count != 0 || other.lost != 0) {
            if (!send_row(other)) {
                return;
            }
            for (uint8_t i = sent; i < active; i++) {
                MethodStats &slot = _stats[slot_of[order[i]]];
                if ((slot.key & SYSTEM_KEY) == 0) {
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
                if ((slot.key & SYSTEM_KEY) != 0) {
                    clear(slot);
                }
            }
        }
    }
    CountersMsg counters {};
    fill_header(counters.header, MsgType::Counters, sizeof(counters), ++_seq);
    counters.mirror_hits = _mirror_hits - _reported_mirror_hits;
    counters.mirror_misses = _mirror_misses - _reported_mirror_misses;
    counters.post_drops = _post_drops - _reported_post_drops;
    if (_channel->send(&counters, sizeof(counters)) != 0) {
        return;   // the next step sends them with the next window's
    }
    _reported_mirror_hits = _mirror_hits;
    _reported_mirror_misses = _mirror_misses;
    _reported_post_drops = _post_drops;
    _last_stats_ms = now_ms;
}

void M4Service::corrupt_tx_ring()
{
    // an unaligned producer index the M7's ring reader rejects (SpscRing::desynced)
    auto *control = reinterpret_cast<Channel::Control *>(reinterpret_cast<uintptr_t>(_window) + PeerHeader::kSize);
    control->m4ToM7.head.value.store(0x7FFFFFF1U);
}

// a token the mirror can hold: module addresses are 32-bit on the M4 image
static bool token_fits(uint64_t token)
{
    return uint64_t(uintptr_t(token)) == token;
}

bool M4Service::mirror_get(uint16_t module, uint64_t token, uint8_t *out5)
{
    _platform.lock();
    const bool hit = module < MAX_MODULES && token_fits(token) && _mirror.get(uint8_t(module), uintptr_t(token), out5);
    if (hit) {
        _mirror_hits++;
    } else {
        _mirror_misses++;
    }
    _platform.unlock();
    return hit;
}

void M4Service::mirror_pending(uint16_t module, uint64_t token)
{
    _platform.lock();
    if (module < MAX_MODULES && token_fits(token)) {
        _mirror.mark_pending(uint8_t(module), uintptr_t(token));
    }
    _platform.unlock();
}

void M4Service::mirror_settle(uint16_t module, uint64_t token)
{
    _platform.lock();
    if (module < MAX_MODULES && token_fits(token)) {
        _mirror.settle(uint8_t(module), uintptr_t(token));
    }
    _platform.unlock();
}

void M4Service::mirror_refresh(uint16_t module, uint64_t token, const uint8_t *out5)
{
    float value;
    memcpy(&value, out5 + 1, sizeof(value));
    _platform.lock();
    if (module < MAX_MODULES && token_fits(token)) {
        _mirror.refresh(uint8_t(module), uintptr_t(token), value, out5[0] != 0);
    }
    _platform.unlock();
}

void M4Service::handle_parameter_changed(const uint8_t *buf, uint16_t len, const MsgHeader &header)
{
    ParameterChangedHeader changed;
    if (!_have_session || header.seq != _session || !parse_parameter_changed(buf, len, changed)) {
        _protocol_errors++;
        return;
    }
    for (uint16_t i = 0; i < changed.count; i++) {
        const ParameterValue entry = parameter_value_at(buf, i);
        if (entry.module >= MAX_MODULES || !token_fits(entry.token)) {
            _protocol_errors++;
            continue;
        }
        if ((_ended_modules & (1U << entry.module)) != 0) {
            continue;   // the M7 keeps an ended module's slots until its session resets
        }
        _mirror.upsert(uint8_t(entry.module), uintptr_t(entry.token), entry.value,
                       (entry.flags & PARAMETER_VALUE_BOUND) != 0);
    }
}

// a module's Returned, Panicked or Failed on its way to the M7: its handles are gone
void M4Service::forget_ended_module()
{
    ModuleStateNotify notify;
    if (_link->request_len != sizeof(notify)) {
        return;
    }
    memcpy(&notify, _link->request, sizeof(notify));
    if (notify.call.method != uint16_t(SystemMethod::ModuleState) || notify.module >= MAX_MODULES) {
        return;
    }
    switch (ModuleStateKind(notify.state)) {
    case ModuleStateKind::Returned:
    case ModuleStateKind::Panicked:
    case ModuleStateKind::Failed:
        _mirror.remove_module(uint8_t(notify.module));
        _ended_modules |= uint8_t(1U << notify.module);
        break;
    case ModuleStateKind::Loaded:
    case ModuleStateKind::Started:
        _ended_modules &= uint8_t(~(1U << notify.module));
        break;
    }
}

}  // namespace RustM4
