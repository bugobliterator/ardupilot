#pragma once

/*
  Per-binding rate limits on expensive M7 work a Rust module on the M4 asks
  for: token buckets per class, refilled at per_second up to burst. Each
  quota'd binding is in a class; over its quota a void binding is dropped,
  like a statustext that finds its queue full, and a boolean binding returns
  false, its in-process failure value.
  The M7 host thread or its slow worker, one at a time. C++11, no
  ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ModuleM4 {

enum class QuotaClass : uint8_t {
    None = 0,
    Text = 1,      // gcs.send_text, gcs.send_named_*, printf
    Storage = 2,   // param set_and_save, mission.set_item, mission.clear
    Led = 3,       // serial_led.send
    FileLog = 4,   // logger.log_file_content: each call queues an M7 heap allocation
    FileIo = 5,    // file writes and removes: SD card writes
    Can = 6,       // CAN frames of write_frame, DroneCAN broadcast and request: the bus flight traffic uses
    MavTx = 7,     // mavlink send: messages on the vehicle's links, under their channel locks
    Log = 8,       // logger.write: rows in the vehicle's log
    Osd = 9,       // osd write, clear: characters on the OSD screen
    OsdFrame = 10, // osd flush, draw_screen: a whole-screen render each
};
static const uint8_t QUOTA_CLASSES = 11;

struct QuotaLimit {
    uint16_t per_second;
    uint16_t burst;
};
static const QuotaLimit QUOTA_LIMITS[QUOTA_CLASSES] = {
    { 0, 0 },        // None: unlimited
    { 20, 40 },      // Text
    { 10, 100 },     // Storage: a module's mission upload fits the burst
    { 100, 100 },    // Led
    { 1, 5 },        // FileLog: the logger drains its file queue at most about 10 files/s
    { 50, 100 },     // FileIo: at most about 25 KB/s of SD writes at FILE_CHUNK per write
    { 800, 800 },    // Can: frames, about 10 % of a 1 Mbit/s bus
    { 200, 200 },    // MavTx: in-tree Lua scripts send under 50 messages a second
    { 500, 500 },    // Log: in-tree Lua actuator_chirp writes 400 rows a second, plane_aerobatics 280
    { 300, 300 },    // Osd: a 30-row screen redrawn at 10 Hz
    { 40, 40 },      // OsdFrame: twice a flush and draw at 10 Hz; the OSD thread draws at 10 Hz
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

    // gives back cost units allow() took for work the M7 then refused (a DroneCAN transfer
    // canard did not queue), up to the burst; the hits are unchanged
    void refund(QuotaClass quota, uint16_t cost)
    {
        const uint8_t index = uint8_t(quota);
        if (quota == QuotaClass::None || index >= QUOTA_CLASSES) {
            return;
        }
        Bucket &bucket = _buckets[index];
        const uint32_t cap = uint32_t(QUOTA_LIMITS[index].burst) * 1000;
        const uint32_t refunded = bucket.millicalls + uint32_t(cost) * 1000;
        bucket.millicalls = refunded > cap ? cap : refunded;
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

}  // namespace ModuleM4
