# AP_Periph Serial Logger Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** An AP_Periph build for CubeOrange+ that captures a raw UART byte stream at up to 12.5 Mbaud with RTS/CTS and writes it unmodified to `LOGnnnnn.TXT` on the microSD.

**Architecture:** Reader thread owns the UART and drains the driver ring into a large DMA-safe `ByteBuffer`; writer thread pushes 32 KB blocks through `AP::FS()` and fsyncs once a second. Two tiny UART driver edits allow a 1 KB RX bounce buffer. A bench mode synthesises a counter stream so the card path can be qualified without an FC.

**Tech Stack:** ArduPilot AP_Periph (C++), ChibiOS HAL, AP_Filesystem/FATFS, waf. Host testing on `sitl_periph_universal` with `--extra-hwdef`.

**Spec:** `Tools/AP_Periph/docs/2026-09-10-serial-logger-design.md`

## Global Constraints

- Feature guard name: `AP_PERIPH_SERIAL_LOGGER_ENABLED` (hwdef `define`, bare `#if` like the other periph features).
- Param group prefix `SLOG_`, names `PORT`, `BAUD`, `BUF_KB`, `BENCH_KBPS`; defaults 1, 5000000, 512, 0 (baud default changed 12500000 → 6250000 → 5000000 on 2026-09-10; 5 M is exact on both a 100 MHz and a 120 MHz UART clock).
- Files in card root on ChibiOS (`/LOG%05u.TXT`, `/LASTLOG.TXT`); under `slog/` on SITL.
- Fixed tuning: 32 KB write block, 1 s fsync, 500 ms write timeout, 16 KB read chunk, 64 MB minimum free space, 3 s remount retry. Each carries a `// ponytail:` marker.
- Board ID for the hwdef: `AP_HW_CUBEORANGEPLUS` (shared with the vehicle firmware).
- Commit author must be `bugobliterator <siddharthbharatpurohit@gmail.com>`; verify `git config user.email` first. Work on branch `periph-serial-logger`.
- Variable names spell out their meaning (`ring_high_water_bytes`, not `hw`).

---

### Task 1: UART driver: overridable RX bounce size

**Files:**
- Modify: `libraries/AP_HAL_ChibiOS/UARTDriver.h:25`
- Modify: `libraries/AP_HAL_ChibiOS/UARTDriver.cpp:1150`

**Interfaces:**
- Produces: hwdef may `define RX_BOUNCE_BUFSIZE <n>`; all receive paths handle n up to 65535.

- [x] **Step 1: Guard the define**

Replace in `UARTDriver.h`:
```c
#define RX_BOUNCE_BUFSIZE 64U
```
with
```c
#ifndef RX_BOUNCE_BUFSIZE
#define RX_BOUNCE_BUFSIZE 64U
#endif
```

- [x] **Step 2: Widen the idle-path length**

In `UARTDriver.cpp` inside `UARTDriver::_timer_tick` (the block starting `if (!enabled) {` near line 1150) change
```c
            uint8_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(rxdma);
```
to
```c
            uint16_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(rxdma);
```

- [x] **Step 3: Build an existing periph target to prove nothing broke**

Run: `./waf configure --board CubeOrange-periph >/dev/null && ./waf AP_Periph 2>&1 | tail -3`
Expected: `'AP_Periph' finished successfully`

- [x] **Step 4: Commit**

```bash
git checkout -b periph-serial-logger
git add libraries/AP_HAL_ChibiOS/UARTDriver.h libraries/AP_HAL_ChibiOS/UARTDriver.cpp
git commit -m "AP_HAL_ChibiOS: allow hwdef override of RX_BOUNCE_BUFSIZE"
```

---

### Task 2: SerialLogger class, params and wiring (logger compiles, does nothing yet)

**Files:**
- Create: `Tools/AP_Periph/serial_logger.h`
- Create: `Tools/AP_Periph/serial_logger.cpp`
- Modify: `Tools/AP_Periph/AP_Periph.h:47` (include) and `:440` (member)
- Modify: `Tools/AP_Periph/AP_Periph.cpp:152` (init call)
- Modify: `Tools/AP_Periph/Parameters.h:108` (enum) and `Tools/AP_Periph/Parameters.cpp:768` (GOBJECT)
- Create (outside repo, scratchpad): `slog_sitl.hwdef` with one line `define AP_PERIPH_SERIAL_LOGGER_ENABLED 1`

**Interfaces:**
- Produces: `class SerialLogger { void init(); AP_Int8 port; AP_Int32 baud; AP_Int16 buf_kb; AP_Int16 bench_kbps; }`, member `periph.serial_logger`, params `SLOG_PORT/BAUD/BUF_KB/BENCH_KBPS`.

- [x] **Step 1: Write the header**

`Tools/AP_Periph/serial_logger.h`:
```cpp
#pragma once

#if AP_PERIPH_SERIAL_LOGGER_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/utility/RingBuffer.h>

/*
  raw serial-to-SD logger (OpenLager replacement). One UART in, one
  LOGnnnnn.TXT out, no framing. See docs/2026-09-10-serial-logger-design.md
 */
class SerialLogger {
public:
    SerialLogger();
    void init();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8  port;        // SERIALn index, -1 disables
    AP_Int32 baud;        // raw baud rate
    AP_Int16 buf_kb;      // big ring size in KB
    AP_Int16 bench_kbps;  // 0 = read UART; else synthesise a counter stream at this rate

private:
    void rx_thread();
    void wr_thread();
    bool open_next_file();
    void close_file();
    bool write_block(uint32_t max_bytes);
    void discard_ring();
    void report_status();
    void bench_generate();

    AP_HAL::UARTDriver *uart;
    ByteBuffer *ring;
    int fd = -1;
    bool mounted;
    uint16_t log_number;

    // status counters, cumulative since boot
    uint32_t ring_high_water_bytes;
    uint32_t write_errors;
    uint32_t bytes_written;
    uint32_t bytes_written_at_last_status;

    uint32_t last_status_ms;
    uint32_t last_fsync_ms;
    uint32_t last_write_ms;
    uint32_t last_mount_attempt_ms;
    uint32_t last_open_attempt_ms;

    // bench generator state
    uint32_t bench_next_value;
    uint32_t bench_last_ms;
    uint32_t bench_carry_bytes;
};

#endif // AP_PERIPH_SERIAL_LOGGER_ENABLED
```

- [x] **Step 2: Write the params and a no-op init**

`Tools/AP_Periph/serial_logger.cpp`:
```cpp
/*
  raw serial-to-SD logger for AP_Periph
 */
#include "AP_Periph.h"

#if AP_PERIPH_SERIAL_LOGGER_ENABLED

#include "serial_logger.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo SerialLogger::var_info[] {
    // @Param: PORT
    // @DisplayName: Serial logger input port
    // @Description: SERIALn index of the UART to capture. -1 disables the logger. The port's SERIALn_PROTOCOL must be -1 (unused).
    // @Range: -1 7
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("PORT", 1, SerialLogger, port, 1),

    // @Param: BAUD
    // @DisplayName: Serial logger baud rate
    // @Description: Raw baud rate of the input port. Up to 12500000 on STM32H7 boards.
    // @Range: 9600 12500000
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("BAUD", 2, SerialLogger, baud, 5000000),

    // @Param: BUF_KB
    // @DisplayName: Serial logger buffer size
    // @Description: Size in KB of the RAM ring buffer that absorbs SD card write stalls. Halved automatically if it cannot be allocated.
    // @Range: 64 768
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("BUF_KB", 3, SerialLogger, buf_kb, 512),

    // @Param: BENCH_KBPS
    // @DisplayName: Serial logger bench rate
    // @Description: When non-zero the logger ignores the UART receive path and writes a synthetic 32-bit counter stream at this rate in KB/s, also transmitting it on the port's TX. Used to qualify SD cards and for loopback tests.
    // @Range: 0 4000
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("BENCH_KBPS", 4, SerialLogger, bench_kbps, 0),

    AP_GROUPEND
};

SerialLogger::SerialLogger()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void SerialLogger::init()
{
}

#endif // AP_PERIPH_SERIAL_LOGGER_ENABLED
```

- [x] **Step 3: Wire into AP_Periph**

`AP_Periph.h`, after `#include "serial_options.h"`:
```cpp
#include "serial_logger.h"
```
`AP_Periph.h`, after the `serial_options` member block:
```cpp
#if AP_PERIPH_SERIAL_LOGGER_ENABLED
    SerialLogger serial_logger;
#endif
```
`AP_Periph.cpp`, after the `serial_options.init();` block:
```cpp
#if AP_PERIPH_SERIAL_LOGGER_ENABLED
    serial_logger.init();
#endif
```
`Parameters.h`, append to the enum after `k_param_battery_bms,`:
```cpp
        k_param_serial_logger,
```
`Parameters.cpp`, after the `GOBJECT(battery_bms, ...)` block, before `AP_VAREND`:
```cpp
#if AP_PERIPH_SERIAL_LOGGER_ENABLED
    // @Group: SLOG_
    // @Path: serial_logger.cpp
    GOBJECT(serial_logger, "SLOG_",  SerialLogger),
#endif
```

- [x] **Step 4: Build SITL periph with the feature on**

```bash
printf 'define AP_PERIPH_SERIAL_LOGGER_ENABLED 1\n' > "$SCRATCH/slog_sitl.hwdef"
./waf configure --board sitl_periph_universal --extra-hwdef="$SCRATCH/slog_sitl.hwdef" >/dev/null && ./waf AP_Periph 2>&1 | tail -3
```
Expected: `'AP_Periph' finished successfully`

- [x] **Step 5: Check the params exist**

Run: `build/sitl_periph_universal/bin/AP_Periph --help >/dev/null; grep -c SLOG_ build/sitl_periph_universal/apj_tool.py 2>/dev/null; strings build/sitl_periph_universal/bin/AP_Periph | grep -E '^(PORT|BAUD|BUF_KB|BENCH_KBPS)$' | sort -u`
Expected: the four param names.

- [x] **Step 6: Commit**

```bash
git add Tools/AP_Periph/serial_logger.h Tools/AP_Periph/serial_logger.cpp Tools/AP_Periph/AP_Periph.h Tools/AP_Periph/AP_Periph.cpp Tools/AP_Periph/Parameters.h Tools/AP_Periph/Parameters.cpp
git commit -m "AP_Periph: add SerialLogger skeleton and SLOG_ params"
```

---

### Task 3: Writer thread, file handling, bench generator, verifier

**Files:**
- Modify: `Tools/AP_Periph/serial_logger.cpp`
- Create: `Tools/AP_Periph/docs/slog_verify.py`

**Interfaces:**
- Consumes: Task 2 class and params.
- Produces: `wr_thread()`, `open_next_file()`, `close_file()`, `write_block()`, `discard_ring()`, `report_status()`, `bench_generate()`; `slog_verify.py <file>` exits 0 on a gap-free counter stream, and `slog_verify.py --generate <bytes> <file>` writes one.

- [x] **Step 1: Write the verifier (the failing test)**

`Tools/AP_Periph/docs/slog_verify.py`:
```python
#!/usr/bin/env python3
"""Check a serial-logger file written in bench mode: a little-endian uint32 counter
starting anywhere and incrementing by one. Also generates such a stream.

  slog_verify.py LOG00001.TXT            -> exit 0 if gap-free
  slog_verify.py --generate N out.bin    -> N bytes of counter stream
"""
import struct, sys

def generate(nbytes, path, start=0):
    with open(path, "wb") as f:
        n = nbytes // 4
        f.write(b"".join(struct.pack("<I", (start + i) & 0xFFFFFFFF) for i in range(n)))

def verify(path):
    data = open(path, "rb").read()
    if len(data) < 8:
        print(f"{path}: too short ({len(data)} bytes)"); return 1
    words = struct.unpack(f"<{len(data)//4}I", data[:len(data)//4*4])
    missing, first_gap = 0, None
    for i in range(1, len(words)):
        step = (words[i] - words[i-1]) & 0xFFFFFFFF
        if step != 1:
            missing += (step - 1) * 4
            if first_gap is None:
                first_gap = i * 4
    print(f"{path}: {len(data)} bytes, first gap at {first_gap}, missing {missing} bytes")
    return 0 if first_gap is None else 1

if __name__ == "__main__":
    if sys.argv[1] == "--generate":
        generate(int(sys.argv[2]), sys.argv[3]); sys.exit(0)
    sys.exit(verify(sys.argv[1]))
```

- [x] **Step 2: Run the verifier on itself to prove it detects gaps**

```bash
python3 Tools/AP_Periph/docs/slog_verify.py --generate 4096 "$SCRATCH/good.bin"
python3 Tools/AP_Periph/docs/slog_verify.py "$SCRATCH/good.bin"; echo "exit $?"
python3 - <<'PY'
import os; s=os.environ["SCRATCH"]; d=open(s+"/good.bin","rb").read(); open(s+"/bad.bin","wb").write(d[:1000]+d[1100:])
PY
python3 Tools/AP_Periph/docs/slog_verify.py "$SCRATCH/bad.bin"; echo "exit $?"
```
Expected: `exit 0` then `first gap at 1000, missing 100 bytes` and `exit 1`.

- [x] **Step 3: Implement init, writer thread and bench generator**

Replace the empty `init()` in `serial_logger.cpp` with the following and add the helpers below it:
```cpp
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define SLOG_DIR "slog"
#else
#define SLOG_DIR ""
#endif

// ponytail: fixed tuning; promote to params only if the SD bench shows they matter
#define SLOG_WRITE_BLOCK_BYTES   32768U
#define SLOG_FSYNC_MS            1000U
#define SLOG_WRITE_TIMEOUT_MS    500U
#define SLOG_READ_CHUNK_BYTES    16384U
#define SLOG_MIN_FREE_BYTES      (64U*1024U*1024U)
#define SLOG_MOUNT_RETRY_MS      3000U
#define SLOG_OPEN_RETRY_MS       1000U
#define SLOG_STATUS_MS           1000U
#define SLOG_MIN_RING_BYTES      (64U*1024U)

void SerialLogger::init()
{
    if (port < 0) {
        return;
    }
    uart = hal.serial(port);
    if (uart == nullptr) {
        can_printf("SLOG: no serial%d", int(port));
        return;
    }

    uint32_t ring_bytes = uint32_t(MAX(int16_t(64), buf_kb.get())) * 1024U;
    while (ring == nullptr && ring_bytes >= SLOG_MIN_RING_BYTES) {
        uint8_t *mem = (uint8_t *)hal.util->malloc_type(ring_bytes, AP_HAL::Util::MEM_DMA_SAFE);
        if (mem != nullptr) {
            ring = NEW_NOTHROW ByteBuffer(mem, ring_bytes);
        }
        if (ring == nullptr) {
            ring_bytes /= 2;
        }
    }
    if (ring == nullptr) {
        can_printf("SLOG: ring alloc failed");
        return;
    }
    can_printf("SLOG: ring %u KB", unsigned(ring_bytes / 1024U));

    mounted = AP::FS().retry_mount();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP::FS().mkdir(SLOG_DIR);
#endif

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SerialLogger::wr_thread, void), "SLOG_WR", 4096, AP_HAL::Scheduler::PRIORITY_IO, 0) ||
        !hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&SerialLogger::rx_thread, void), "SLOG_RX", 2048, AP_HAL::Scheduler::PRIORITY_CAN, 1)) {
        can_printf("SLOG: thread create failed");
    }
}

/*
  writer: unmounted -> closed -> open. Never lets the ring stay full
  because of a missing card, so RTS only drops for genuine stalls.
 */
void SerialLogger::wr_thread()
{
    while (true) {
        const uint32_t now_ms = AP_HAL::millis();

        if (!mounted) {
            if (now_ms - last_mount_attempt_ms >= SLOG_MOUNT_RETRY_MS) {
                last_mount_attempt_ms = now_ms;
                mounted = AP::FS().retry_mount();
            }
            discard_ring();
            hal.scheduler->delay(10);
            report_status();
            continue;
        }

        if (fd == -1) {
            if (ring->available() > 0 && now_ms - last_open_attempt_ms >= SLOG_OPEN_RETRY_MS) {
                last_open_attempt_ms = now_ms;
                if (!open_next_file()) {
                    discard_ring();
                }
            }
            hal.scheduler->delay(10);
            report_status();
            continue;
        }

        const uint32_t avail = ring->available();
        if (avail >= SLOG_WRITE_BLOCK_BYTES ||
            (avail > 0 && now_ms - last_write_ms >= SLOG_WRITE_TIMEOUT_MS)) {
            if (!write_block(SLOG_WRITE_BLOCK_BYTES)) {
                write_errors++;
                close_file();
                mounted = false;
            }
        } else {
            hal.scheduler->delay(1);
        }

        if (fd != -1 && now_ms - last_fsync_ms >= SLOG_FSYNC_MS) {
            last_fsync_ms = now_ms;
            AP::FS().fsync(fd);
        }
        report_status();
    }
}

// write one contiguous run from the ring; returns false on a write error
bool SerialLogger::write_block(uint32_t max_bytes)
{
    ByteBuffer::IoVec vec[2];
    const uint8_t nvec = ring->peekiovec(vec, max_bytes);
    if (nvec == 0) {
        return true;
    }
    const int32_t written = AP::FS().write(fd, vec[0].data, vec[0].len);
    if (written <= 0) {
        return false;
    }
    ring->advance(written);
    bytes_written += written;
    last_write_ms = AP_HAL::millis();
    return true;
}

void SerialLogger::discard_ring()
{
    ring->advance(ring->available());
}

bool SerialLogger::open_next_file()
{
    if (AP::FS().disk_free(SLOG_DIR "/") < int64_t(SLOG_MIN_FREE_BYTES)) {
        return false;
    }

    // next number from LASTLOG.TXT, same scheme as AP_Logger_File
    uint16_t last = 0;
    int lfd = AP::FS().open(SLOG_DIR "/LASTLOG.TXT", O_RDONLY);
    if (lfd != -1) {
        char buf[16] {};
        if (AP::FS().read(lfd, buf, sizeof(buf) - 1) > 0) {
            last = strtoul(buf, nullptr, 10);
        }
        AP::FS().close(lfd);
    }
    log_number = last + 1;

    char fname[32];
    snprintf(fname, sizeof(fname), SLOG_DIR "/LOG%05u.TXT", unsigned(log_number));
    fd = AP::FS().open(fname, O_WRONLY | O_CREAT | O_TRUNC);
    if (fd == -1) {
        return false;
    }

    lfd = AP::FS().open(SLOG_DIR "/LASTLOG.TXT", O_WRONLY | O_CREAT | O_TRUNC);
    if (lfd != -1) {
        char buf[16];
        const int len = snprintf(buf, sizeof(buf), "%u\r\n", unsigned(log_number));
        AP::FS().write(lfd, buf, len);
        AP::FS().close(lfd);
    }

    last_write_ms = last_fsync_ms = AP_HAL::millis();
    can_printf("SLOG: opened %s", fname);
    return true;
}

void SerialLogger::close_file()
{
    if (fd != -1) {
        AP::FS().close(fd);
        fd = -1;
    }
}

void SerialLogger::report_status()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_status_ms < SLOG_STATUS_MS) {
        return;
    }
    const uint32_t dt_ms = now_ms - last_status_ms;
    last_status_ms = now_ms;
    const uint32_t rate_kbps = (bytes_written - bytes_written_at_last_status) / MAX(dt_ms, 1U);
    bytes_written_at_last_status = bytes_written;
    const uint32_t size = ring->get_size();
    can_printf("SLOG: rate=%u KB/s ring=%u%% hw=%u%% drop=%u werr=%u file=%u",
               unsigned(rate_kbps),
               unsigned(ring->available() * 100U / size),
               unsigned(ring_high_water_bytes * 100U / size),
               unsigned(uart->get_total_dropped_rx_bytes()),
               unsigned(write_errors),
               unsigned(fd == -1 ? 0 : log_number));
}

/*
  bench mode: push a uint32 counter stream into the ring at bench_kbps,
  and mirror it on the UART TX for loopback tests
 */
void SerialLogger::bench_generate()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (bench_last_ms == 0) {
        bench_last_ms = now_ms;
        return;
    }
    const uint32_t dt_ms = now_ms - bench_last_ms;
    bench_last_ms = now_ms;

    uint32_t budget = dt_ms * uint32_t(bench_kbps.get()) * 1024U / 1000U + bench_carry_bytes;
    budget = MIN(budget, ring->space()) & ~3U;
    bench_carry_bytes = 0;

    ByteBuffer::IoVec vec[2];
    const uint8_t nvec = ring->reserve(vec, budget);
    uint32_t total = 0;
    for (uint8_t i = 0; i < nvec; i++) {
        const uint32_t nwords = vec[i].len / 4U;
        uint8_t *p = vec[i].data;
        for (uint32_t w = 0; w < nwords; w++, p += 4) {
            p[0] = bench_next_value & 0xFF;
            p[1] = (bench_next_value >> 8) & 0xFF;
            p[2] = (bench_next_value >> 16) & 0xFF;
            p[3] = (bench_next_value >> 24) & 0xFF;
            bench_next_value++;
        }
        uart->write(vec[i].data, nwords * 4U);
        total += nwords * 4U;
    }
    ring->commit(total);
    bench_carry_bytes = budget - total;
}

void SerialLogger::rx_thread()
{
    uart->begin(baud, 65535, bench_kbps > 0 ? 4096 : 0);
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE);

    while (true) {
        if (bench_kbps > 0) {
            bench_generate();
            hal.scheduler->delay(1);
        } else {
            hal.scheduler->delay(1);   // replaced by the real reader in Task 4
        }
        ring_high_water_bytes = MAX(ring_high_water_bytes, ring->available());
    }
}
```
Add `#include <stdlib.h>` and `#include <fcntl.h>` at the top of the file with the other includes (`strtoul`, `O_*` flags).

- [x] **Step 4: Build SITL periph**

Run: `./waf AP_Periph 2>&1 | grep -E "error|warning: unused|finished" | head`
Expected: only `'AP_Periph' finished successfully`

- [x] **Step 5: Run bench mode on SITL and verify the file**

```bash
cd "$SCRATCH" && rm -rf slog && printf 'SLOG_BENCH_KBPS 200\nSLOG_PORT 1\n' > slog.parm
timeout 8 "$REPO/build/sitl_periph_universal/bin/AP_Periph" --defaults slog.parm --serial1=none 2>&1 | grep SLOG | head -12
ls -l slog/; python3 "$REPO/Tools/AP_Periph/docs/slog_verify.py" slog/LOG00001.TXT; echo "exit $?"
cat slog/LASTLOG.TXT
```
Expected: status lines with `rate=` near 200, `werr=0`; `LOG00001.TXT` about 1.2–1.6 MB; verifier `first gap at None, missing 0 bytes`, `exit 0`; `LASTLOG.TXT` contains `1`. A second run must create `LOG00002.TXT`.

- [x] **Step 6: Commit**

```bash
git add Tools/AP_Periph/serial_logger.cpp Tools/AP_Periph/docs/slog_verify.py
git commit -m "AP_Periph: SerialLogger writer thread, file handling and bench mode"
```

---

### Task 4: Reader thread (real UART path)

> Execution note (2026-09-10): the SITL loopback below was attempted and dropped at the user's
> request; the periph SITL main loop needs a copter SITL multicast feed and the pty path is not
> representative. Verification moves to the hardware loopback in Task 6.

**Files:**
- Modify: `Tools/AP_Periph/serial_logger.cpp` (`rx_thread`)

**Interfaces:**
- Consumes: Task 3 ring, writer, verifier.
- Produces: bytes arriving on `hal.serial(SLOG_PORT)` end up in `LOGnnnnn.TXT`; reader pauses when the ring has under 16 KB free.

- [ ] **Step 1: Write the failing test: feed a counter stream through a pty into SITL**

```bash
cd "$SCRATCH" && rm -rf slog
python3 "$REPO/Tools/AP_Periph/docs/slog_verify.py" --generate 2000000 feed.bin
socat -d -d pty,raw,echo=0,link=ptyA pty,raw,echo=0,link=ptyB 2>/dev/null &
SOCAT=$!; sleep 1
printf 'SLOG_BENCH_KBPS 0\nSLOG_PORT 1\nSLOG_BAUD 921600\n' > slog_rx.parm
(timeout 15 "$REPO/build/sitl_periph_universal/bin/AP_Periph" --defaults slog_rx.parm --serial1=uart:$PWD/ptyA 2>&1 | grep SLOG | tail -4) &
sleep 3; cat feed.bin > ptyB; sleep 6; wait
kill $SOCAT
python3 "$REPO/Tools/AP_Periph/docs/slog_verify.py" slog/LOG00001.TXT; echo "exit $?"
```
Expected before implementation: no `slog/LOG00001.TXT` (verifier errors), because `rx_thread` only sleeps.

- [x] **Step 2: Implement the reader loop**

Replace the `else` branch in `rx_thread` with:
```cpp
        } else {
            if (ring->space() < SLOG_READ_CHUNK_BYTES) {
                // ring nearly full: stop reading so the driver ring fills and RTS drops
                hal.scheduler->delay(1);
            } else {
                if (!uart->wait_timeout(1, 5)) {
                    // HALs without wait support (SITL) return false immediately
                    hal.scheduler->delay_microseconds(500);
                }
                ByteBuffer::IoVec vec[2];
                const uint8_t nvec = ring->reserve(vec, MIN(SLOG_READ_CHUNK_BYTES, ring->space()));
                uint32_t total = 0;
                for (uint8_t i = 0; i < nvec; i++) {
                    const ssize_t n = uart->read(vec[i].data, MIN(vec[i].len, 65535U));
                    if (n <= 0) {
                        break;
                    }
                    total += n;
                    if (uint32_t(n) < vec[i].len) {
                        break;
                    }
                }
                ring->commit(total);
            }
        }
```

- [ ] **Step 3: Build and rerun the loopback test**

Run: `./waf AP_Periph 2>&1 | tail -1` then the Step 1 commands again.
Expected: `LOG00001.TXT` is 2000000 bytes, verifier `missing 0 bytes`, `exit 0`.

- [x] **Step 4: Commit**

```bash
git add Tools/AP_Periph/serial_logger.cpp
git commit -m "AP_Periph: SerialLogger UART reader thread"
```

---

### Task 5: CubeOrangePlus-periph hwdef and firmware build

**Files:**
- Create: `libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-periph/hwdef.dat`

**Interfaces:**
- Consumes: Task 1 override, Task 2 feature guard.
- Produces: `build/CubeOrangePlus-periph/bin/AP_Periph.apj` flashable with the stock CubeOrange+ bootloader.

- [x] **Step 1: Write the hwdef**

```
# CubeOrange+ as a raw serial logger (OpenLager replacement)
include ../CubeOrangePlus/hwdef.dat

undef IOMCU_UART
undef USART6
undef ROMFS
undef HAL_HAVE_SAFETY_SWITCH
undef HAL_CHIBIOS_ARCH_FMUV3
undef BOOTLOADER_DEV_LIST
undef BOARD_VALIDATE
undef IMU

# same board ID as the vehicle firmware so the stock bootloader accepts it
APJ_BOARD_ID AP_HW_CUBEORANGEPLUS
# stock bootloader is kept; no embedded bootloader image for this target
define AP_BOOTLOADER_FLASHING_ENABLED 0

# setup build for a peripheral firmware
env AP_PERIPH 1

define AP_CAN_SLCAN_ENABLED 1
define AP_PERIPH_SERIAL_LOGGER_ENABLED 1
define HAL_CAN_DRIVER_DEFAULT 1
define HAL_GPIO_PIN_LED HAL_GPIO_PIN_FMU_LED_AMBER

# per-port byte counters, used by the SLOG status line (dropped bytes)
define HAL_UART_STATS_ENABLED 1

# listen for reboot command from uploader.py on the USB port
define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0

# the logger needs the card: SDMMC1 and HAL_OS_FATFS_IO stay from the base hwdef

# 1 KB RX DMA bounce buffers: ISR every 0.8 ms at 1.25 MB/s instead of every 51 us
define RX_BOUNCE_BUFSIZE 1024

# TELEM1 RX DMA stream is never shared and wins arbitration
DMA_NOSHARE USART2*
DMA_PRIORITY USART2*

MAIN_STACK 0x2000
PROCESS_STACK 0x6000
```

- [x] **Step 2: Build**

Run: `./waf configure --board CubeOrangePlus-periph >/dev/null && ./waf AP_Periph 2>&1 | grep -E "error|Memory|Free|finished" | head`
Expected: `'AP_Periph' finished successfully`; note the reported flash/RAM figures in the commit message body.

- [x] **Step 3: Check the generated hwdef honours the override and features**

Run: `grep -E "RX_BOUNCE_BUFSIZE|AP_PERIPH_SERIAL_LOGGER_ENABLED|HAL_OS_FATFS_IO|STM32_SDC_USE_SDMMC1|HAL_LOGGING_ENABLED" build/CubeOrangePlus-periph/hwdef.h`
Expected: `RX_BOUNCE_BUFSIZE 1024`, logger enabled, FATFS IO 1, SDMMC1 TRUE, no `HAL_LOGGING_ENABLED 1`.

- [x] **Step 4: Commit**

```bash
git add libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-periph/hwdef.dat
git commit -m "hwdef: add CubeOrangePlus-periph serial logger target"
```

---

### Task 6: Hardware bench gate (needs the board; documented, not automated)

**Files:** none (records results in the design spec's Open items when done).

- [x] **Step 1: Flash** — done over J-Link with OpenOCD (`program AP_Periph.bin 0x08020000 verify`), stock bootloader kept; Cube USB was not attached so uploader.py was not used.
- [x] **Step 2: Bench gate (one card, 625 kB/s, 231 s, 2026-09-10):** 148.6 MB at 643 kB/s, `werr=0`, longest write 15 ms, ring high-water 16 %. Param injected at a GDB breakpoint on `SerialLogger::init`; counters read over GDB. Remaining: other cards, 1250 kB/s, 10 min.
- [ ] **Step 3: Loopback:** jumper TELEM1 TX to RX, default `SLOG_BAUD` (6.25 Mbaud) first and then `SLOG_BAUD 12500000`, bench mode on; pull the card, run `slog_verify.py` on the file. Pass: zero gaps. Note that TX-side rate is capped by the 64-byte TX bounce buffer.
- [x] **Step 3b: CTS back-pressure against the real FC (2026-09-10 05:34 UTC, firmware `57bc120344`):** Betaflight CubeRacer streaming ~96.8 kB/s at 6.25 Mbaud into `LOG00004.TXT`; two 2,000 µs RTS pulses at T0+10 s and T0+20 s. Firmware snapshots: RX DMA NDTR 1018 → 1024 and 1019 → 1024 (fresh buffer after the idle flush, nothing arrived during the pulse), RX count +7 and +6 (buffered bytes plus one in flight). ~190 bytes would have landed per pulse had CTS been ignored. Stream counters at 05:35:16: 3,087,620 received, 0 dropped, 0 write errors, high-water 13 % of 256 KB, longest write 6 ms.
- [x] **Step 3c: Sustained real stream (2026-09-10 05:39–05:42 UTC):** three minutes of Betaflight blackbox at ~98 kB/s appended to `LOG00004.TXT`. Received = written = 20,736,322 bytes (17,632,252 this run), 0 UART drops, 0 write errors, ring high-water 13 % of 256 KB, longest write 6 ms; 30 s counter series in the sync file. The card never stalled long enough to raise CTS; a slower card or a card pull is needed to exercise the ring beyond 13 %.
- [x] **Step 3d: Decode failure and receive-path bug (2026-09-10 05:52–06:22 UTC):** the Betaflight decoder found single characters missing from the blackbox header in LOG00003–5 while received = written and all counters were zero. Root cause on the Cube: the ChibiOS USARTv3 serial ISR drains RDR on every IDLE interrupt with no check that RXNEIE is on, so a byte the DMA had not fetched yet went into the unused SerialDriver input queue. Fixed with `SerialConfig.external_rx_buffer` (ChibiOS submodule `d6ee2d6865`, USARTv1/2/3) set by the ChibiOS UART driver when RX DMA is on; also skip the F4 IDLE-clearing DR read when a byte is pending.
- [x] **Step 3e: Receive path exonerated (2026-09-10 07:40–08:02 UTC):** TX-to-RX loopback with an on-board word checker (`SLOG_BENCH_LOOP 1`), card writing throughout: 64-byte chunks at 625 kB/s (46.9 MB, 0 breaks), 1-byte / 3-byte / 16-byte chunks with the inter-chunk gap swept 1–40 µs (2.0 / 6.8 / 33.7 MB, 0 breaks, 0 drops, 0 ORE/FE/NE). FC passthrough counter stream over the same UART4 (`SLOG_BENCH_LOOP 2`): 327,680 bytes exact, 81,919 words, 0 breaks.
- [x] **Step 3f: Fault located on the FC (2026-09-10 08:30–09:34 UTC):** with the FC counting bytes accepted and DMA-scheduled, every Blackbox capture arrives short at the Cube (3, 7, 37,516 and TBD bytes) with 0 Cube errors and RTS never raised, at 6.25 Mbaud with CTS on or off and at 5 Mbaud. Header deletions are contiguous runs. TRBUFF (H7 errata) added on the FC did not fix it. **Confirmed 10:06 UTC:** with the FC tracing its TX DMA starts, 873 starts found the previous transfer unfinished (EN clear, old NDTR nonzero) and reprogrammed NDTR while the tail had already been advanced, discarding 1,066 bytes; the Cube's deficit for that capture (accepted 3,203,161 − received 3,202,095) was exactly 1,066, with every Cube-side counter zero (drops, ORE/FE/NE, RTS raised, NDTR moved in the completion ISR, DMA error flags). Fix is on the Betaflight side.
- [x] **Step 3g: USB log retrieval (2026-09-10 09:30 UTC):** `dump_request` streams a closed log over the USB console with a CRC32 trailer; LOG00012 pulled at 563 KB/s with an exact SHA256 match to the card copy, LOG00018 at 828 KB/s.
- [x] **Step 3h: End-to-end acceptance (2026-09-10 10:26–10:40 UTC, FC with its DMA-starter flag-clear fix):** LOG00020, 30 s: FC accepted = DMA-scheduled = Cube received = 3,206,290; first 1 KiB identical to the sender prefix; official `blackbox_decode` 105,921 consecutive frames, no gaps. LOG00021, 180 s on the FC's trace-free build with two 2 ms RTS pulses at T0+30/90 s: received = written = 17,722,706, 0 drops / ORE / FE / NE / ndtr_moved / dma_err, header identical to the sender prefix, ring high-water 19 % through a 181 ms card write. Both files pulled over the Cube's USB at ~850 KB/s with CRC.
- [x] **Step 4: Record** ring high-water and the chosen card: 512 KB could not be allocated, the ring runs at 256 KB; worst card stall seen 181 ms, absorbed at 19 % high-water (50,331 bytes), so `SLOG_BUF_KB 512` stays as the request and the halving fallback is the working configuration on this card.
