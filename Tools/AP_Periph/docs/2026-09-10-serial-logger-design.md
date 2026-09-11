# AP_Periph serial logger — design spec

Date 2026-09-10. Companion to `serial_logger_viability.md`. Target: CubeOrange+ running
AP_Periph as an OpenLager replacement, capturing a Betaflight blackbox byte stream at up to
12.5 Mbaud with RTS/CTS flow control and writing it raw to microSD.

## 1. Goals and non-goals

Goals

- Receive on one Cube UART at any baud up to 12.5 Mbaud (1.25 MB/s payload) with zero
  dropped bytes while the card keeps up, and clean RTS back-pressure when it does not.
- Write the stream unmodified to `LOGnnnnn.TXT`, one file per stream (closed after 5 s of silence), readable directly by
  Betaflight Blackbox tools.
- Configuration through ordinary AP_Periph parameters over USB SLCAN or DroneCAN.
- Built-in bench mode so the card and the full write path can be qualified without a
  separate test build.

Non-goals

- Anything on the Betaflight side (baud table, OVER8, CTS handling). Handled separately.
- Other AP_Periph features (GPS, compass, baro, IMU, MAVLink). This is a logger-only build.
- Structured `.BIN` logging, per-flight file splitting, SD config files, pre-allocation.
- Rates above 12.5 Mbaud (would need the 120 MHz USART clock change described in the
  viability study).

## 2. Architecture

```
TELEM1 (USART2) --DMA--> 1 KB bounce x2 --ISR--> driver ring 64 KB
        ^ RTS                                         |
        |                                    reader thread (owner)
        +--- ISR drops RTS when ring < 4 KB free      v
                                          big ring, SLOG_BUF_KB (512 KB, AXI SRAM)
                                                      |
                                              writer thread
                                                      v
                                   AP::FS() -> FATFS -> SDMMC1 IDMA -> card
```

Two threads and one ring buffer. The reader never calls the filesystem; the writer never
touches the UART. Back-pressure is three-stage: big ring absorbs card stalls up to
~410 ms at 1.25 MB/s; when it is full the reader stops reading; the driver ring then fills
and the existing RX interrupt code drops RTS about 3 ms before overflow.

## 3. Components

### 3.1 `Tools/AP_Periph/serial_logger.{h,cpp}` (new, ~150 lines)

Class `SerialLogger`, instantiated in `AP_Periph_FW` under `#if AP_PERIPH_SERIAL_LOGGER_ENABLED`.

`init()` (called from `AP_Periph_FW::init`)

1. Return if `SLOG_PORT < 0` or `hal.serial(SLOG_PORT)` is null.
2. Allocate the big ring: `ByteBuffer` sized `SLOG_BUF_KB * 1024` from
   `hal.util->malloc_type(..., MEM_FILESYSTEM)` (the AXI SRAM heap on H7; `MEM_DMA_SAFE` is the 64 KB SRAM4 heap and cannot hold it). On failure halve until it fits or reaches
   64 KB; report the final size on the console.
3. `AP::FS().retry_mount()` once.
4. Start the two threads with `hal.scheduler->thread_create`:
   reader `"SLOG_RX"` at base `PRIORITY_CAN`, offset +1, 2 KB stack; writer `"SLOG_WR"` at
   `PRIORITY_IO`, 4 KB stack.

Reader thread (`rx_thread`)

- Owns the UART: calls `begin(SLOG_BAUD, 65535, 0)` and
  `set_flow_control(FLOW_CONTROL_ENABLE)` itself so `_uart_owner_thd` is this thread.
  The H7 UART FIFO stays enabled (driver default).
- Loop: if the big ring has under 16 KB free, `delay(1)` and continue (this is what lets
  RTS drop). Else `wait_timeout(1, 5)`, then `read()` up to 16 KB directly into the
  ring's write `IoVec` (no intermediate copy), `commit()`.
- Records ring high-water mark and reads the driver's dropped-byte stat via
  `get_total_dropped_rx_bytes()` for the status line.
- Bench mode (`SLOG_BENCH_KBPS > 0`): instead of reading the UART, generate a
  little-endian `uint32_t` counter stream at the requested rate (per-millisecond budget,
  carry remainder) into the ring, and also `write()` the same bytes to the UART TX for
  the loopback test. The UART is still opened so RX loopback works.

Writer thread (`wr_thread`)

- State: `unmounted`, `closed`, `open`. Timers: last remount attempt, last fsync.
- `unmounted`: every 3 s `AP::FS().retry_mount()`; on success go to `closed`. Meanwhile
  drain the ring (discard) so RTS never sticks low because of a missing card.
- `closed`: when the ring has data, open the next file (see 3.2) immediately. On failure stay
  `closed`, retry once a second, keep draining. On success go to `open`, and reset the
  per-file metrics (ring high-water, longest write).
- `open`: wait until the ring holds ≥ 32 KB or 500 ms passed since the last write, then
  `peekbytes` one contiguous `IoVec` (≤ 32 KB, so one `f_write` and one multi-block SD
  command), `AP::FS().write()`, `advance()` by the bytes written. `fsync` when 1 s has
  elapsed since the last one. A write error after FATFS's own remount retry closes the
  file, increments `write_errors`, and returns to `unmounted`.
- Both thresholds carry `// ponytail:` markers; they become params only if the bench shows
  they matter.

Status: once a second on the console and, when CAN is up, as a `uavcan.protocol.debug.LogMessage` via `can_printf`:
`SLOG: rate=%u KB/s ring=%u%% hw=%u%% stall=%ums drop=%u werr=%u file=%u`, where `stall` is the
longest single filesystem write since the file was opened (the SD stall measure) and
`hw` the ring high-water since open. Also printed on the USB console.

### 3.2 Files

- Root of the card, `LOGnnnnn.TXT`. Next number = value in `LASTLOG.TXT` + 1, then write
  the new number back (same scheme as `AP_Logger_File`). Missing or unreadable
  `LASTLOG.TXT` starts at 1.
- Opened on first byte, not at boot. Flags `O_WRONLY|O_CREAT|O_TRUNC`. Closed after 5 s
  with no bytes (OpenLager style, decided 2026-09-10 after the bench showed back-to-back
  runs sharing a file); the next byte opens the next number. So one file per stream, not
  per boot.
- Before opening, `disk_free()` under 64 MB is treated like an open failure (card full).

### 3.3 Parameters (`Tools/AP_Periph/Parameters.cpp`, group `SLOG_`)

| Name | Type | Default | Range | Meaning |
|---|---|---|---|---|
| `SLOG_PORT` | AP_Int8 | 1 | -1..7 | SERIALn index of the input port; -1 disables |
| `SLOG_BAUD` | AP_Int32 | 5000000 | 9600..12500000 | raw baud rate, not a SerialManager table index. Default 5 Mbaud (agreed 2026-09-10) is exact on both the Cube's 100 MHz UART clock (divider 20) and a 120 MHz FC clock (divider 24), 16x oversampled; 12.5 Mbaud is the ceiling |
| `SLOG_BUF_KB` | AP_Int16 | 512 | 64..768 | big ring size in KB |
| `SLOG_BENCH_KBPS` | AP_Int16 | 0 | 0..4000 | 0 = normal; else synthesise a counter stream at this rate (KB/s) |

`SERIALn_PROTOCOL` for the chosen port must be -1 (unused) so SerialManager does not
reconfigure it. `SLOG_BENCH_KBPS` is reboot-required.

### 3.4 Driver changes (`libraries/AP_HAL_ChibiOS/UARTDriver`)

1. `UARTDriver.h`: wrap `#define RX_BOUNCE_BUFSIZE 64U` in `#ifndef` so a hwdef can
   override it.
2. `UARTDriver.cpp`, idle/timer receive path (~line 1150): `uint8_t len` → `uint16_t len`.
   The ISR path already uses `uint16_t`. Without this, a bounce buffer over 255 bytes
   truncates silently.
3. `AP_HAL/UARTDriver.h`: the cumulative byte-count getters (`get_total_dropped_rx_bytes`
   and friends) move from `protected` to `public` so the logger can report drops.

Nothing else changes: OVER8 switch above 6.25 Mbaud, DMA restart order, software RTS with
threshold `ring/16` and hardware CTS are used as they are.

### 3.5 hwdef `libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-periph/hwdef.dat` (new)

```
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

env AP_PERIPH 1

define AP_CAN_SLCAN_ENABLED 1
define AP_PERIPH_SERIAL_LOGGER_ENABLED 1
define HAL_CAN_DRIVER_DEFAULT 1
define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0
define HAL_GPIO_PIN_LED HAL_GPIO_PIN_FMU_LED_AMBER

# per-port byte counters, used by the SLOG status line (dropped bytes)
define HAL_UART_STATS_ENABLED 1

# logger needs the card: keep SDMMC1 and HAL_OS_FATFS_IO from the base hwdef

# 1 KB RX DMA bounce buffers: ISR rate 1.2 kHz at 1.25 MB/s instead of 19.5 kHz
define RX_BOUNCE_BUFSIZE 1024

DMA_NOSHARE USART2*
DMA_PRIORITY USART2*

MAIN_STACK 0x2000
PROCESS_STACK 0x6000
```

No `AP_PERIPH_*_ENABLED` features other than the logger, no `HAL_LOGGING_ENABLED`.
A `hwdef-bl.dat` is not needed because the board ID is shared with the stock bootloader.

## 4. Data flow and sizing at 12.5 Mbaud

| Stage | Size | Time at 1.25 MB/s | Notes |
|---|---|---|---|
| USART FIFO | 16 B | 12.8 µs | covers the DMA restart gap in the ISR |
| RX bounce buffer | 1 KB ×2 | 0.8 ms each | ISR every 0.8 ms |
| Driver ring | 64 KB | 52 ms | RTS drops at 4 KB free (3.3 ms before overflow) |
| Big ring | 512 KB | 410 ms | absorbs card stalls; reader pauses at 16 KB free |
| Write block | 32 KB | 26 ms | one `f_write`, one multi-block SD write |
| fsync | every 1 s | | bounded loss on power cut |

RAM: 512 KB ring + 12 KB bounce buffers + 64 KB driver ring, out of the H757's 1 MB,
with the ring placed in the 512 KB AXI SRAM region the RAM map reserves for SDMMC IDMA.
`init()` halves the ring on allocation failure, so a smaller heap degrades rather than
fails.

## 5. Error handling

| Condition | Behaviour |
|---|---|
| Card absent at boot | writer in `unmounted`, retries every 3 s, ring drained, RTS stays high |
| Card removed while logging | write error → close → `unmounted`; on reinsertion a new file is opened |
| Card full | open refused under 64 MB free; treated as open failure, retried every 1 s |
| SD stall > 410 ms | big ring fills, reader pauses, driver ring fills, RTS drops; FC drops frames, stream stays intact |
| Ring alloc failure | halve until ≥ 64 KB; log final size (on CubeOrange+ the AXI heap gave 256 KB, not 512 KB) |
| Port not DMA-capable (e.g. UART4 RX) | works but at reduced rate; console warning if `sdef.dma_rx` is false |
| `SLOG_PORT` out of range or null | logger disabled, console message |

All counters (`dropped`, `write_errors`, high-water) are cumulative since boot and
reported in the status line.

## 6. Testing

1. **Bench gate** (before trusting any card): `SLOG_BENCH_KBPS 1250`, no FC attached,
   10 minutes, three cards. Pass: high-water < 60 %, `werr = 0`, throughput within 1 % of
   1250 KB/s. Run this first; it decides whether the 32 KB block and 1 s fsync defaults
   stand.
2. **Loopback**: same bench mode with a TX-to-RX jumper on TELEM1 at
   `SLOG_BAUD 12500000`. Exercises DMA RX, bounce buffers and RTS at full rate. TX may be
   slower than 1.25 MB/s because of the 64-byte TX bounce buffer; correctness is the aim.
3. **Verifier** (`Tools/AP_Periph/docs/slog_verify.py`, the one runnable check): reads a log
   file, checks the `uint32_t` counter is contiguous, prints first gap offset and total
   missing bytes. Pass: zero gaps.
4. **FC end to end** with the user's patched Betaflight at 12.5 Mbaud: `blackbox_decode`
   frame count equals the FC's; pulling the card mid-run drops RTS rather than corrupting
   the stream.
5. **Soak**: 30 minutes at 8 kHz, `blackbox_sample_rate 1/1`, decode in Blackbox Explorer.

## 7. Files touched

| File | Change |
|---|---|
| `Tools/AP_Periph/serial_logger.h/.cpp` | new |
| `Tools/AP_Periph/AP_Periph.h/.cpp` | member, `init()` call, feature guard |
| `Tools/AP_Periph/Parameters.h/.cpp` | `SLOG_` group |
| `Tools/AP_Periph/wscript` | add source (if not globbed) |
| `libraries/AP_HAL_ChibiOS/UARTDriver.h` | `#ifndef RX_BOUNCE_BUFSIZE` |
| `libraries/AP_HAL_ChibiOS/UARTDriver.cpp` | `uint8_t len` → `uint16_t` |
| `libraries/AP_HAL_ChibiOS/hwdef/CubeOrangePlus-periph/hwdef.dat` | new |
| `Tools/AP_Periph/docs/slog_verify.py` | new |

## 8. Open items

- Confirm on hardware that `hal.util->available_memory()` leaves ≥ 512 KB after init;
  otherwise lower `SLOG_BUF_KB` default. Measured 2026-09-10: 512 KB is refused, the
  ring runs at 256 KB (about 410 ms at 625 kB/s); the worst card stall seen so far,
  178 ms, was absorbed at 19 % high-water.
- Bench-only additions made during hardware debug and kept: `SLOG_BENCH_CHUNK`,
  `SLOG_BENCH_LOOP` (loopback / verify-only counter check), RTS pulse request with
  NDTR snapshots, USB log dump (`dump_request`). None of them run unless requested.
- Receive path: the ChibiOS serial ISR must not read RDR while the DMA owns the
  receiver; fixed via `SerialConfig.external_rx_buffer` (ChibiOS submodule). Verified
  by loopback at 1/3/16/64-byte chunks and by an FC-generated counter stream, all
  byte-exact. Remaining blackbox byte loss is on the FC transmit side (see plan 3f).
- Signal integrity of TELEM1 at 12.5 Mbaud through the carrier board (scope). Fallback is
  10 Mbaud, also exact on the stock clock.
