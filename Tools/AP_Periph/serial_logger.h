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
    AP_Int16 bench_chunk; // bench only: 0 = 1 ms bursts; else one unbuffered write of this
                          // many bytes per wire chunk, gap swept 1..40 us between chunks
    AP_Int8  bench_loop;  // bench only: 1 = transmit the counter on TX and fill the ring from
                          // the normal receive path (TX-to-RX jumper), so the file holds
                          // what actually came back

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
    uint32_t max_write_ms;      // longest single AP::FS().write() call, the SD stall measure

    // bench: raise the RTS (FC CTS) line for this many microseconds, then clear.
    // Written from a debugger or console; the reader thread services it and zeroes it.
    volatile uint32_t rts_pulse_request_us;
    uint32_t rts_pulses_done;
    // snapshots around the last pulse: RX DMA NDTR and UART RX byte count taken just
    // before raising the line and just before lowering it, plus the raise time
    uint32_t rts_pulse_ndtr_before;
    uint32_t rts_pulse_ndtr_after;
    uint32_t rts_pulse_rx_before;
    uint32_t rts_pulse_rx_after;
    uint32_t rts_pulse_time_ms;

    uint32_t last_status_ms;
    uint32_t last_fsync_ms;
    uint32_t last_write_ms;
    uint32_t last_mount_attempt_ms;
    uint32_t last_open_attempt_ms;

    // bench generator state
    uint32_t bench_next_value;
    uint32_t bench_last_ms;
    uint32_t bench_carry_bytes;
    uint32_t bench_gap_phase;
    uint32_t bench_tx_short;    // loopback: writes that did not take every byte offered
    uint8_t  bench_tx_byte_index;  // loopback: next byte of bench_next_value to send (0..3)
    uint32_t fill_counter(ByteBuffer::IoVec *vec, uint8_t nvec, bool transmit);
    void bench_transmit();

    // stream a finished log over the USB console port on request from the
    // debugger (write the file number here), so it can be copied without
    // pulling the card. Serviced by the writer thread while no file is open.
    void dump_file(uint16_t number);
    volatile uint16_t dump_request;
    bool     dumping;
    uint32_t dump_bytes_sent;
    uint32_t dump_errors;
    uint8_t *dump_buf;

    // loopback: on-board check of the received counter stream, so a run can be
    // judged over the debugger without pulling the card
    void bench_verify(const uint8_t *data, uint32_t len);
    uint8_t  vfy_buf[8];
    uint8_t  vfy_len;
    bool     vfy_synced;
    bool     vfy_resyncing;
    uint32_t vfy_expected;
    uint32_t vfy_rx_offset;          // bytes seen by the checker
    uint32_t vfy_words_ok;
    uint32_t vfy_mismatches;         // sequence breaks (each is at least one lost byte)
    uint32_t vfy_first_bad_offset;   // rx offset of the first break
    uint32_t vfy_last_bad_offset;
};

#endif // AP_PERIPH_SERIAL_LOGGER_ENABLED
