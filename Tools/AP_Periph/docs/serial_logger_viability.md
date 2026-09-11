# AP_Periph serial blackbox logger on CubeOrange+ — viability study

Target: replace an OpenLager with a CubeOrange+ (STM32H757, FMU only) running AP_Periph,
receiving Betaflight blackbox over one UART and writing the raw byte stream to microSD at
**≥ 12 Mbit/s**. Study date 2026-09-10, against ArduPilot master `63ac2d68f2`.

## 1. Verdict

**Viable on the Cube side with two small firmware changes; the USART clock stays stock because 12.5 Mbaud is accepted as the design point.** The two open risks are
outside ArduPilot: Betaflight's serial stack tops out at 2.47 Mbaud today, and sustained
microSD write speed must be bench-measured, not assumed.**

| Area | Status | What is needed |
|---|---|---|
| UART receive at 12.5 Mbaud | Feasible, stock clock | Bigger RX DMA bounce buffer only; 12.5 Mbaud is exact on the 100 MHz USART clock |
| Buffering against SD stalls | Feasible | 512 KB ring in AXI SRAM, two threads |
| microSD sustained 1.25 MB/s | Unproven | Bench test with a synthetic stream before writing any product code |
| Betaflight transmit at 12.5 Mbaud | Not in stock BF | New baud table entry, OVER8 on the FC UART; AT32/G4/H7 land within 1.1 %, F722 is marginal at +1.6 % |
| Electrical link at 12.5 Mbaud | Unproven | Scope the RX line through the Cube carrier; keep cable short |

## 2. What "12 Mbit/s" means

UART framing is 10 bits per byte. **Decision (2026-09-10): design point is 12.5 Mbaud**,
the exact rate the stock 100 MHz USART clock produces. That is 12.5 Mbit/s on the wire
and 1.25 MB/s (10 Mbit/s) of blackbox payload. Everything below is sized for 1.25 MB/s.

For scale: Betaflight at an 8 kHz PID loop with `blackbox_sample_rate 1/1` emits on the
order of a few Mbit/s (rough estimate, not measured here). 12 Mbit/s is headroom, not a
current Betaflight requirement.

## 3. UART receive path on CubeOrange+

### 3.1 Port choice

`SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7 OTG2`
(`libraries/AP_HAL_ChibiOS/hwdef/CubeOrange/hwdef.inc:32`). RX DMA is mandatory at these
rates, so:

| Port | Cube label | RX DMA | Notes |
|---|---|---|---|
| USART2 (SERIAL1) | TELEM1 | yes | RTS/CTS available. **Recommended.** |
| USART3 (SERIAL2) | TELEM2 | yes | RTS/CTS available. Second choice. |
| UART4 (SERIAL3) | GPS1 | **no** (`NODMA` on RX) | Unusable |
| UART8 (SERIAL4) | GPS2 | yes (TX is NODMA) | Usable for RX-only |
| UART7 (SERIAL5) | | yes | Usable |

USART6 goes to the IOMCU and is freed in the periph build (`undef IOMCU_UART`), but it is
not brought out on a connector.

### 3.2 Clock and achievable baud rates

The H7 USART kernel clock is PLL2_Q
(`libraries/AP_HAL_ChibiOS/hwdef/common/stm32h7_mcuconf.h:378-379`). With the Cube's
24 MHz crystal: 24 / DIVM 2 × DIVN 50 / DIVQ 6 = **100 MHz**. The driver switches to 8×
oversampling above 6.25 Mbaud (`libraries/AP_HAL_ChibiOS/UARTDriver.cpp:447`), giving a
hard ceiling of **12.5 Mbaud**.

ChibiOS computes the divider on the 16× basis and then doubles it for OVER8
(`modules/ChibiOS/os/hal/ports/STM32/LLD/USARTv3/hal_serial_lld.c:282-288`), so at these
rates only `100 MHz / integer` is reachable. Result (from `baud_check.py`; the 120 MHz row is
kept for reference only):

| USART kernel clock | ask 12.0 Mbaud | ask 12.5 Mbaud | ask 15.0 Mbaud |
|---|---|---|---|
| CubeOrange+ 100 MHz (stock PLL2_Q) | 12.500 M (+4.2 %) | 12.500 M (+0.0 %) | not reachable |
| CubeOrange+ 120 MHz (PLL2 DIVQ 6 → 5) | 12.000 M (+0.0 %) | 12.000 M (−4.0 %) | 15.000 M (+0.0 %) |
| BF STM32F405 APB2 84 MHz | not reachable | not reachable | not reachable |
| BF STM32F722 APB2 108 MHz | 12.000 M (+0.0 %) | 12.706 M (+1.6 %) | not reachable |
| BF STM32H743 pclk 120 MHz | 12.000 M (+0.0 %) | 12.632 M (+1.1 %) | 15.000 M (+0.0 %) |
| BF STM32G474 170 MHz | 12.143 M (+1.2 %) | 12.593 M (+0.7 %) | 14.783 M (−1.4 %) |
| BF AT32F435 APB2 144 MHz | 12.000 M (+0.0 %) | 12.522 M (+0.2 %) | 15.158 M (+1.1 %) |

Betaflight rows assume the FC UART driver is patched to OVER8 with ST-HAL rounding. With
8× oversampling the receiver tolerates roughly ±2 % total mismatch. **At 12.5 Mbaud the stock
clock is exact and no clock change is needed.** The mismatch budget is then spent entirely
on the Betaflight FC: AT32F435 (+0.2 %), G474 (+0.7 %) and H743 (+1.1 %) are fine; F722
(+1.6 %) leaves under 0.5 % for crystal error and should be scoped before trusting it.

If 15 Mbaud is ever wanted, the path is PLL2 DIVQ 6 → 5 (120 MHz), which also moves the
SPI4/5/6 kernel clock and needs an `#ifndef` guard added in `stm32h7_mcuconf.h`. Not
part of this plan.

### 3.3 DMA and buffer path

Received bytes flow: USART FIFO (16 B) → DMA into a 64-byte bounce buffer
(`RX_BOUNCE_BUFSIZE`, `UARTDriver.h:25`) → ISR copies into `_readbuf` → reader thread.

| Item | Stock | At 1.25 MB/s | Change |
|---|---|---|---|
| Bounce buffer | 64 B | ISR every 51 µs (19.5 kHz) | **Change 1:** guard the define, set 1024 B in this hwdef → 1.2 kHz |
| DMA restart gap | covered by 16 B FIFO | 12.8 µs of slack at 12.5 Mbaud | none; check no higher-priority ISR exceeds this |
| `_readbuf` size | `uint16_t rxS` in `begin()` → max 64 KB | 52 ms of data | too small to ride out SD stalls on its own |

The ISR restarts DMA before copying (`UARTDriver.cpp:610-627`), which is the right order;
dropped bytes are counted in `_rx_stats_dropped_bytes`, which the logger should export.

### 3.4 Electrical

Bit time at 12.5 Mbaud is 80 ns on a 3.3 V push-pull line that crosses the Cube's DF17
connector and the carrier board. The knowledgebase has no note on series resistors or
ESD parts on the TELEM lines, so this is a bench item: scope RX at the Cube with a
< 20 cm cable and a solid ground, check rise time and eye. Keep the default 3-sample
majority (do not set `ONEBIT`).

## 4. microSD write path

Hardware: SDMMC1, 4-bit, clock capped at 50 MHz (`stm32h7_mcuconf.h:696`), so the bus
is ~25 MB/s. The bottleneck is the card, not the bus.

Software facts:

- `AP_Filesystem_FATFS::write` hands a buffer straight to `f_write` if it is DMA-safe,
  otherwise chops it into `io_size` pieces (4–32 KB bounce buffer, `sdcard.cpp:73-80`).
  So the ring buffer must live in AXI SRAM (`MEM_DMA_SAFE`), which is also the region
  the RAM map reserves for SDMMC IDMA (`STM32H757xx.py:36`).
- `AP_Logger_File` writes 4 KB chunks once per io-thread tick and fsyncs on `io_size`
  boundaries. That path is tuned for structured `.BIN` logs and is not the shape we want
  here; it is also the wrong container (Betaflight tools need the raw stream).
- FATFS is built with `FF_FS_EXFAT 1`, `FF_USE_EXPAND 0`. Enabling `FF_USE_EXPAND` and
  pre-allocating a contiguous file removes FAT chain updates from the hot path.

Card requirement: 1.25 MB/s sustained is well inside an A1/A2 card's rated sequential
write, but consumer cards stall for tens to hundreds of ms during internal housekeeping.
A 512 KB ring holds 410 ms at 1.25 MB/s. **Plan on a 512 KB ring in AXI SRAM.** The H757 has 1 MB of RAM and AP_Periph uses a small fraction of it, so
this fits; confirm with `hal.util->available_memory()` on the target.

**This is the gate.** No published number exists in the repo for sustained FATFS write
throughput on this board, so step 1 of the test plan measures it.

## 5. Proposed software shape

One new file in `Tools/AP_Periph`, about 150 lines, plus params:

```
uart (owner thread A) --read()--> ByteBuffer 512 KB (AXI SRAM) --thread B--> AP::FS().write()
```

- **Thread A** (`PRIORITY_CAN` or higher): `begin(baud, 65535, 0)`, loop on
  `wait_timeout(1, 5)` then `read()` into the ring. Never blocks on the filesystem.
- **Thread B** (`PRIORITY_IO`): waits until ≥ 32 KB is available, writes one 32 KB block
  (one `f_write`, one multi-block SD command), fsyncs once per second. Opens
  `LOGnnnnn.TXT` on first byte, like OpenLager; closes after 5 s of silence.
- **Params:** `SLOG_PORT`, `SLOG_BAUD` (uint32, not the SerialManager baud table),
  `SLOG_BUF_KB`. Health: dropped-byte counter and high-water mark, on the console and
  as a DroneCAN debug message.
- **hwdef (Change 2):** new `CubeOrangePlus-periph` copying the `CubeOrange-periph`
  pattern but keeping `SDMMC1` and `HAL_OS_FATFS_IO` (as `CubeOrange-periph-heavy` already
  does), plus the bounce-buffer override. No `CubeOrangePlus-periph` target exists today.

Rejected: routing through `AP_Logger` (wrong container, 4 KB writes, extra framing) and
a single-thread design (a 300 ms SD stall would overflow the 64 KB UART ring in 52 ms).

Two threads and a ring buffer are the whole design. `// ponytail:` markers go on the
fixed 32 KB block size and the 1 s fsync interval; both become params only if the bench
shows they matter.

## 6. Betaflight side (out of scope: handled separately by the user)

Listed for completeness only; no Cube-side work depends on it.

- Stock `baudRates[]` ends at 2 470 000 (`src/main/io/serial.c`), which is the real
  reason OpenLager users sit at ≤ 2.47 Mbaud. A new entry is a one-line change plus the
  enum.
- The FC UART driver must run OVER8 and use TX DMA on that port; F405 cannot reach 12.5
  Mbaud at all (table above). AT32F435, G474 and H743 land within 1.1 %; F722 is marginal.
- Betaflight has no hardware flow control on the blackbox port, so the Cube must absorb
  every stall itself (hence the 512 KB ring).
- Betaflight docs recommend only 250 kbaud for OpenLog and note OpenLog is unusable
  past 4.0; nothing there constrains the Cube design.

## 7. Risks

| # | Risk | Likelihood | Mitigation |
|---|---|---|---|
| 1 | Card stalls exceed buffer | medium | measure; A2 card; pre-allocated contiguous file; larger ring |
| 2 | Signal integrity at 12.5 Mbaud | medium | scope; short cable; fall back to 10 Mbaud (100 MHz / 10, also exact) |
| 3 | ISR latency at UART priority spikes past 12.8 µs | low | 1024 B bounce; keep CAN and SD IRQ priorities as-is |
| 4 | FC baud error (F722 +1.6 %) eats the OVER8 tolerance | medium | prefer AT32/G4/H7 FC; verify with scope |

## 8. Test plan (do step 1 before writing product code)

1. **SD bench on target:** a throwaway build that fills a 512 KB ring at 1.25 MB/s from a
   timer and drains it with 32 KB writes. Record throughput, max single-write latency,
   ring high-water mark over 10 minutes with three cards. Pass: high-water < 60 %.
2. **UART loopback at 12.5 Mbaud:** TELEM1 TX → RX with `SLOG_BAUD 12500000` on stock
   firmware clocks; confirm zero `_rx_stats_dropped_bytes` over 1 GB.
3. **Electrical:** scope RX at the Cube with the intended cable; then FC → Cube with a
   patched Betaflight, compare decoded frame counts to the FC's frame counter.
4. **Soak:** 30 min flight-length run, verify log decodes in Blackbox Explorer.

## 9. Effort

| Item | Days |
|---|---|
| SD bench firmware + measurements | 1.5 |
| bounce-size override (guarded define) | 0.25 |
| `CubeOrangePlus-periph` hwdef | 0.5 |
| `serial_logger.cpp` + params | 1.5 |
| Loopback, electrical, soak tests | 2 |

Not included: the Betaflight patch.

## Sources

- ArduPilot master `63ac2d68f2`, files cited inline.
- Betaflight `src/main/io/serial.c` `baudRates[]`;
  Betaflight docs, [Blackbox logging and usage](https://betaflight.com/docs/wiki/guides/current/Black-Box-logging-and-usage);
  [Serial BlackBox Logging wiki](https://github.com/betaflight/betaflight/wiki/Serial-BlackBox-Logging);
  [betaflight#9101](https://github.com/betaflight/betaflight/issues/9101).
- CubePilot knowledgebase: no notes on Cube UART baud limits, SD throughput, or TELEM
  line components (searched 2026-09-10).
