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
    Can = 6,       // CAN frames of write_frame, DroneCAN broadcast and request: the bus flight traffic uses
    MavTx = 7,     // mavlink:send_chan: messages on the vehicle's links, under their channel locks
    Log = 8,       // logger:write: rows in the vehicle's log
    Osd = 9,       // osd:write, osd:clear: characters on the OSD screen
    OsdFrame = 10, // osd:flush, osd:draw_screen: a whole-screen render each
};
static const uint8_t QUOTA_CLASSES = 11;

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
    { 800, 800 },    // Can: frames, about 10 % of a 1 Mbit/s bus
    { 200, 200 },    // MavTx: in-tree scripts send under 50 messages a second
    { 500, 500 },    // Log: actuator_chirp.lua writes 400 rows a second, plane_aerobatics.lua 280
    { 300, 300 },    // Osd: a 30-row screen redrawn at 10 Hz
    { 40, 40 },      // OsdFrame: twice osd.lua's flush and draw at 10 Hz; the OSD thread draws at 10 Hz
};

class Quota {
public:
    // full buckets
    void reset(uint32_t now_ms)
    {
        for (uint8_t i = 0; i < QUOTA_CLASSES; i++) {
            _buckets[i].millicalls = uint32_t(QUOTA_LIMITS[i].burst) * 1000;
            _buckets[i].last_ms = now_ms;
        }
    }

    // a new M4 session: the buckets are filled once per boot and keep their levels across
    // sessions, so a requested restart is not a refill. The hits not yet taken are kept, so
    // a session's last refusals still reach M4ST after a restart
    void new_session(uint32_t now_ms)
    {
        if (!_filled) {
            reset(now_ms);
            _filled = true;
        }
    }

    // true if a call of this class costing cost units (calls; CAN frames) may run now, and
    // takes them from the bucket
    bool allow(QuotaClass quota, uint32_t now_ms, uint16_t cost = 1)
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
        const uint32_t needed = uint32_t(cost) * 1000;
        if (bucket.millicalls < needed) {
            _hits++;
            return false;
        }
        bucket.millicalls -= needed;
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
    bool _filled = false;
};

}  // namespace ScriptingM4
