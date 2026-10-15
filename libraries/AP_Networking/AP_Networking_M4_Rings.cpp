// the M4 image defines AP_NETWORKING_M4_ENABLED on its command line and has no AP_HAL
#ifndef AP_NETWORKING_M4_ENABLED
#include "AP_Networking_Config.h"
#endif

#if AP_NETWORKING_M4_ENABLED

#include "AP_Networking_M4_Layout.h"

namespace NetM4 {

void ByteRing::attach(RingControl *control, uint8_t *data, uint32_t size)
{
    _control = control;
    _data = data;
    _size = size;
    _index_mask = 2 * size - 1;
}

bool ByteRing::desynced() const
{
    const uint32_t head = _control->head.value.load(std::memory_order_acquire);
    const uint32_t tail = _control->tail.value.load(std::memory_order_acquire);
    return head > _index_mask || tail > _index_mask || used(head, tail) > _size;
}

uint32_t ByteRing::space() const
{
    if (desynced()) {
        return 0;
    }
    return _size - used(_control->head.value.load(std::memory_order_relaxed),
                        _control->tail.value.load(std::memory_order_acquire));
}

uint32_t ByteRing::available() const
{
    if (desynced()) {
        return 0;
    }
    return used(_control->head.value.load(std::memory_order_acquire),
                _control->tail.value.load(std::memory_order_relaxed));
}

void ByteRing::copy_in(uint32_t index, const uint8_t *src, uint32_t len)
{
    const uint32_t offset = index & (_size - 1);
    const uint32_t first = len < _size - offset ? len : _size - offset;
    memcpy(&_data[offset], src, first);
    memcpy(_data, src + first, len - first);
}

void ByteRing::copy_out(uint32_t index, uint8_t *dst, uint32_t len) const
{
    const uint32_t offset = index & (_size - 1);
    const uint32_t first = len < _size - offset ? len : _size - offset;
    memcpy(dst, &_data[offset], first);
    memcpy(dst + first, _data, len - first);
}

uint32_t ByteRing::write(const uint8_t *src, uint32_t len)
{
    const uint32_t free_bytes = space();
    if (len > free_bytes) {
        len = free_bytes;
    }
    const uint32_t head = _control->head.value.load(std::memory_order_relaxed);
    copy_in(head, src, len);
    _control->head.value.store((head + len) & _index_mask, std::memory_order_release);
    return len;
}

bool ByteRing::write_record(const uint8_t *src, uint16_t len)
{
    if (len > MAX_RECORD || space() < uint32_t(len) + 2) {
        return false;
    }
    const uint8_t header[2] = { uint8_t(len), uint8_t(len >> 8) };
    const uint32_t head = _control->head.value.load(std::memory_order_relaxed);
    copy_in(head, header, 2);
    copy_in(head + 2, src, len);
    _control->head.value.store((head + 2 + len) & _index_mask, std::memory_order_release);
    return true;
}

void ByteRing::producer_reset()
{
    _control->head.value.store(_control->tail.value.load(std::memory_order_acquire) & _index_mask,
                               std::memory_order_release);
}

uint32_t ByteRing::read(uint8_t *dst, uint32_t len)
{
    const uint32_t have = available();
    if (len > have) {
        len = have;
    }
    const uint32_t tail = _control->tail.value.load(std::memory_order_relaxed);
    copy_out(tail, dst, len);
    _control->tail.value.store((tail + len) & _index_mask, std::memory_order_release);
    return len;
}

int32_t ByteRing::read_record(uint8_t *dst, uint16_t capacity)
{
    if (desynced()) {
        return RECORD_BAD;
    }
    const uint32_t have = available();
    if (have < 2) {
        return RECORD_NONE;
    }
    const uint32_t tail = _control->tail.value.load(std::memory_order_relaxed);
    uint8_t header[2];
    copy_out(tail, header, 2);
    const uint16_t len = uint16_t(header[0] | (header[1] << 8));
    if (len > MAX_RECORD || len > capacity || uint32_t(len) + 2 > have) {
        return RECORD_BAD;
    }
    copy_out(tail + 2, dst, len);
    _control->tail.value.store((tail + 2 + len) & _index_mask, std::memory_order_release);
    return len;
}

void ByteRing::consumer_resync()
{
    _control->tail.value.store(_control->head.value.load(std::memory_order_acquire) & _index_mask,
                               std::memory_order_release);
}

void write_status(StatusBlock &block, const StatusFields &fields)
{
    const uint32_t seq = block.seq.load(std::memory_order_relaxed);
    block.seq.store(seq | 1U, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    memcpy(static_cast<void *>(&block.fields), &fields, sizeof(fields));
    std::atomic_thread_fence(std::memory_order_release);
    block.seq.store((seq | 1U) + 1U, std::memory_order_release);
}

static bool status_values_valid(const StatusFields &fields)
{
    if (fields.magic != STATUS_MAGIC || (fields.flags & ~STATUS_KNOWN_FLAGS) != 0) {
        return false;
    }
    for (uint32_t state : fields.port_state) {
        if (state > PORT_CONNECTED) {
            return false;
        }
    }
    return true;
}

bool read_status(const StatusBlock &block, StatusFields &out)
{
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        const uint32_t before = block.seq.load(std::memory_order_acquire);
        if (before & 1U) {
            continue;
        }
        StatusFields copy;
        memcpy(&copy, &block.fields, sizeof(copy));
        std::atomic_thread_fence(std::memory_order_acquire);
        if (block.seq.load(std::memory_order_relaxed) != before) {
            continue;
        }
        if (!status_values_valid(copy)) {
            return false;
        }
        out = copy;
        return true;
    }
    return false;
}

}  // namespace NetM4

#endif  // AP_NETWORKING_M4_ENABLED
