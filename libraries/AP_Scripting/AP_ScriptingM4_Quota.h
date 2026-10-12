#pragma once

/*
  Per-method rate limits on expensive M7 work the M4 asks for (spec §7):
  token buckets per class, refilled at per_second up to burst. The generator
  puts each quota'd method in a class (gen-bindings -r, remote_quotas[]); over
  its quota a void method is dropped, like a statustext that finds its queue
  full, and a boolean method returns false, its in-process failure value.
  Host thread only. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

enum class QuotaClass : uint8_t {
    None = 0,
    Text = 1,      // gcs:send_text, send_named_*, print
    Storage = 2,   // param:set_and_save, Parameter:set_and_save, mission:set_item, mission:clear
    Led = 3,       // serialLED:send
    FileLog = 4,   // logger:log_file_content: each call queues an M7 heap allocation
    FileIo = 5,    // m4:fs_open for writing, m4:fs_write, m4:fs_remove: SD card writes
    Can = 6,       // CAN write_frame, DroneCAN broadcast and request: the bus flight traffic uses
};
static const uint8_t QUOTA_CLASSES = 7;

struct QuotaLimit {
    uint16_t per_second;
    uint16_t burst;
};
static const QuotaLimit QUOTA_LIMITS[QUOTA_CLASSES] = {
    { 0, 0 },        // None: unlimited
    { 20, 40 },      // Text
    { 10, 100 },     // Storage: a script's mission upload fits the burst
    { 100, 100 },    // Led
    { 1, 5 },        // FileLog: the logger drains its file queue at most about 10 files/s
    { 50, 100 },     // FileIo: at most about 25 KB/s of SD writes at FILE_CHUNK per write
    { 100, 100 },    // Can
};

class Quota {
public:
    // full buckets; the hits not yet taken are kept, so a session's last refusals
    // still reach M4ST after a restart
    void reset(uint32_t now_ms)
    {
        for (uint8_t i = 0; i < QUOTA_CLASSES; i++) {
            _buckets[i].millicalls = uint32_t(QUOTA_LIMITS[i].burst) * 1000;
            _buckets[i].last_ms = now_ms;
        }
    }

    // true if a call of this class may run now (and takes it from the bucket)
    bool allow(QuotaClass quota, uint32_t now_ms)
    {
        const uint8_t index = uint8_t(quota);
        if (quota == QuotaClass::None || index >= QUOTA_CLASSES) {
            return true;
        }
        Bucket &bucket = _buckets[index];
        const QuotaLimit &limit = QUOTA_LIMITS[index];
        // per_second calls per 1000 ms: elapsed ms * per_second thousandths of a call
        const uint64_t refilled = uint64_t(bucket.millicalls) + uint64_t(now_ms - bucket.last_ms) * limit.per_second;
        const uint32_t cap = uint32_t(limit.burst) * 1000;
        bucket.millicalls = refilled > cap ? cap : uint32_t(refilled);
        bucket.last_ms = now_ms;
        if (bucket.millicalls < 1000) {
            _hits++;
            return false;
        }
        bucket.millicalls -= 1000;
        return true;
    }

    // calls refused since the last call (M4ST QHit)
    uint32_t take_hits()
    {
        const uint32_t hits = _hits;
        _hits = 0;
        return hits;
    }

private:
    struct Bucket {
        uint32_t millicalls;   // thousandths of a call
        uint32_t last_ms;
    };
    Bucket _buckets[QUOTA_CLASSES] {};
    uint32_t _hits = 0;
};

}  // namespace ScriptingM4
