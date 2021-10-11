/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && !defined(HAL_NO_UARTDRIVER) && defined(HAL_USE_SERIAL)
#include "UARTDevice.h"
#include "GPIO.h"
#include "shared_dma.h"
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Common/ExpandingString.h>
#include "Scheduler.h"
#include "hwdef/common/stm32_util.h"

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// threshold for disabling TX DMA due to contention
#if defined(USART_CR1_FIFOEN)
#define CONTENTION_BAUD_THRESHOLD 460800
#else
#define CONTENTION_BAUD_THRESHOLD 115200
#endif

UARTDevice::UARTDevice(const UARTDriver::SerialDef &_sdef, ByteBuffer &readbuf, ByteBuffer &writebuf, HAL_Semaphore &write_mutex, uint32_t baudrate) :
sdef(_sdef),
_readbuf(readbuf),
_writebuf(writebuf),
_write_mutex(write_mutex),
_baudrate(baudrate)
{}

bool UARTDevice::open()
{
    if (sdef.serial == nullptr) {
        return false;
    }

#ifndef HAL_UART_NODMA
    if (!half_duplex && !(_last_options & AP_HAL::UARTDriver::OPTION_NODMA_RX)) {
        if (rx_bounce_buf[0] == nullptr && sdef.dma_rx) {
            rx_bounce_buf[0] = (uint8_t *)hal.util->malloc_type(RX_BOUNCE_BUFSIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
        if (rx_bounce_buf[1] == nullptr && sdef.dma_rx) {
            rx_bounce_buf[1] = (uint8_t *)hal.util->malloc_type(RX_BOUNCE_BUFSIZE, AP_HAL::Util::MEM_DMA_SAFE);
        }
    }
    if (tx_bounce_buf == nullptr && sdef.dma_tx && !(_last_options & AP_HAL::UARTDriver::OPTION_NODMA_TX)) {
        tx_bounce_buf = (uint8_t *)hal.util->malloc_type(TX_BOUNCE_BUFSIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
    if (half_duplex) {
        rx_dma_enabled = tx_dma_enabled = false;
    } else {
        rx_dma_enabled = rx_bounce_buf[0] != nullptr && rx_bounce_buf[1] != nullptr;
        tx_dma_enabled = tx_bounce_buf != nullptr;
    }
    if (contention_counter > 1000 && _baudrate <= CONTENTION_BAUD_THRESHOLD) {
        // we've previously disabled TX DMA due to contention, don't
        // re-enable on a new begin() unless high baudrate
        tx_dma_enabled = false;
    }
    if (_baudrate <= 115200 && sdef.dma_tx && Shared_DMA::is_shared(sdef.dma_tx_stream_id)) {
        // avoid DMA on shared low-baudrate links
        tx_dma_enabled = false;
    }
#endif

#if defined(USART_CR1_FIFOEN)
    // enable the UART FIFO on G4 and H7. This allows for much higher baudrates
    // without data loss when not using DMA
    if (_last_options & AP_HAL::UARTDriver::OPTION_NOFIFO) {
        _cr1_options &= ~USART_CR1_FIFOEN;
    } else {
        _cr1_options |= USART_CR1_FIFOEN;
    }
#endif

    if (_baudrate != 0) {
#ifndef HAL_UART_NODMA
        bool was_initialised = _device_initialised;
        // setup Rx DMA
        if (!_device_initialised) {
            if (rx_dma_enabled) {
                osalDbgAssert(rxdma == nullptr, "double DMA allocation");
                chSysLock();
                rxdma = dmaStreamAllocI(sdef.dma_rx_stream_id,
                                        12,  //IRQ Priority
                                        (stm32_dmaisr_t)rxbuff_full_irq,
                                        (void *)this);
                osalDbgAssert(rxdma, "stream alloc failed");
                chSysUnlock();
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
                dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->RDR);
#else
                dmaStreamSetPeripheral(rxdma, &((SerialDriver*)sdef.serial)->usart->DR);
#endif // STM32F7
#if STM32_DMA_SUPPORTS_DMAMUX
                dmaSetRequestSource(rxdma, sdef.dma_rx_channel_id);
#endif
            }
            _device_initialised = true;
        }
        if (tx_dma_enabled && dma_handle == nullptr) {
            // we only allow for sharing of the TX DMA channel, not the RX
            // DMA channel, as the RX side is active all the time, so
            // cannot be shared
            dma_handle = new Shared_DMA(sdef.dma_tx_stream_id,
                                        SHARED_DMA_NONE,
                                        FUNCTOR_BIND_MEMBER(&UARTDevice::dma_tx_allocate, void, Shared_DMA *),
                                        FUNCTOR_BIND_MEMBER(&UARTDevice::dma_tx_deallocate, void, Shared_DMA *));
            if (dma_handle == nullptr) {
                tx_dma_enabled = false;
            }
        }
#endif // HAL_UART_NODMA
        sercfg.speed = _baudrate;

        // start with options from set_options()
        sercfg.cr1 = _cr1_options;
        sercfg.cr2 = _cr2_options;
        sercfg.cr3 = _cr3_options;

#ifndef HAL_UART_NODMA
        if (rx_dma_enabled) {
            sercfg.cr1 |= USART_CR1_IDLEIE;
            sercfg.cr3 |= USART_CR3_DMAR;
        }
        if (tx_dma_enabled) {
            sercfg.cr3 |= USART_CR3_DMAT;
        }
        sercfg.irq_cb = rx_irq_cb;
#endif // HAL_UART_NODMA
        if (!(sercfg.cr2 & USART_CR2_STOP2_BITS)) {
            sercfg.cr2 |= USART_CR2_STOP1_BITS;
        }
        sercfg.ctx = (void*)this;

        sdStop((SerialDriver*)sdef.serial);
        sdStart((SerialDriver*)sdef.serial, &sercfg);

#ifndef HAL_UART_NODMA
        if (rx_dma_enabled) {
            //Configure serial driver to skip handling RX packets
            //because we will handle them via DMA
            ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
            // Start DMA
            if (!was_initialised) {
                dmaStreamDisable(rxdma);
                dma_rx_enable();
            }
        }
#endif // HAL_UART_NODMA
    }
    // Setup Flow Control
    set_flow_control(_flow_control);
    return true;
}

#ifndef HAL_UART_NODMA
void UARTDevice::dma_tx_allocate(Shared_DMA *ctx)
{
    if (txdma != nullptr) {
        return;
    }
    chSysLock();
    txdma = dmaStreamAllocI(sdef.dma_tx_stream_id,
                            12,  //IRQ Priority
                            (stm32_dmaisr_t)tx_complete,
                            (void *)this);
    osalDbgAssert(txdma, "stream alloc failed");
    chSysUnlock();
#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->TDR);
#else
    dmaStreamSetPeripheral(txdma, &((SerialDriver*)sdef.serial)->usart->DR);
#endif // STM32F7
#if STM32_DMA_SUPPORTS_DMAMUX
    dmaSetRequestSource(txdma, sdef.dma_tx_channel_id);
#endif
}

#ifndef HAL_UART_NODMA
void UARTDevice::dma_rx_enable(void)
{
    uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
    dmamode |= STM32_DMA_CR_CHSEL(sdef.dma_rx_channel_id);
    dmamode |= STM32_DMA_CR_PL(0);
#if defined(STM32H7)
    dmamode |= 1<<20;   // TRBUFF See 2.3.1 in the H743 errata
#endif
    rx_bounce_idx ^= 1;
    stm32_cacheBufferInvalidate(rx_bounce_buf[rx_bounce_idx], RX_BOUNCE_BUFSIZE);
    dmaStreamSetMemory0(rxdma, rx_bounce_buf[rx_bounce_idx]);
    dmaStreamSetTransactionSize(rxdma, RX_BOUNCE_BUFSIZE);
    dmaStreamSetMode(rxdma, dmamode | STM32_DMA_CR_DIR_P2M |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(rxdma);
}
#endif

void UARTDevice::dma_tx_deallocate(Shared_DMA *ctx)
{
    chSysLock();
    dmaStreamFreeI(txdma);
    txdma = nullptr;
    chSysUnlock();
}

#ifndef HAL_UART_NODMA
void UARTDevice::rx_irq_cb(void* self)
{
    UARTDevice* uart_dev = (UARTDevice*)self;
    if (!uart_dev->rx_dma_enabled) {
        return;
    }
#if defined(STM32F7) || defined(STM32H7)
    //disable dma, triggering DMA transfer complete interrupt
    uart_dev->rxdma->stream->CR &= ~STM32_DMA_CR_EN;
#elif defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
    //disable dma, triggering DMA transfer complete interrupt
    dmaStreamDisable(uart_dev->rxdma);
    uart_dev->rxdma->channel->CCR &= ~STM32_DMA_CR_EN;
#else
    volatile uint16_t sr = ((SerialDriver*)(uart_dev->sdef.serial))->usart->SR;
    if(sr & USART_SR_IDLE) {
        volatile uint16_t dr = ((SerialDriver*)(uart_dev->sdef.serial))->usart->DR;
        (void)dr;
        //disable dma, triggering DMA transfer complete interrupt
        uart_dev->rxdma->stream->CR &= ~STM32_DMA_CR_EN;
    }
#endif // STM32F7
}
#endif //#ifndef HAL_UART_NODMA

/*
  handle a RX DMA full interrupt
 */
void UARTDevice::rxbuff_full_irq(void* self, uint32_t flags)
{
    UARTDevice* uart_dev = (UARTDevice*)self;
    if (!uart_dev->rx_dma_enabled) {
        return;
    }
    uint16_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(uart_dev->rxdma);
    const uint8_t bounce_idx = uart_dev->rx_bounce_idx;

    // restart the DMA transfers immediately. This switches to the
    // other bounce buffer. We restart the DMA before we copy the data
    // out to minimise the time with DMA disabled, which allows us to
    // handle much higher receiver baudrates
    dmaStreamDisable(uart_dev->rxdma);
    uart_dev->dma_rx_enable();
    
    if (len > 0) {
        /*
          we have data to copy out
         */
        uart_dev->_readbuf.write(uart_dev->rx_bounce_buf[bounce_idx], len);
        uart_dev->receive_timestamp_update();
    }

    if (uart_dev->_wait.thread_ctx && uart_dev->_readbuf.available() >= uart_dev->_wait.n) {
        chSysLockFromISR();
        chEvtSignalI(uart_dev->_wait.thread_ctx, UARTDriver::EVT_DATA);
        chSysUnlockFromISR();
    }
    if (uart_dev->_rts_is_active) {
        uart_dev->update_rts_line();
    }
}
#endif // HAL_UART_NODMA

bool UARTDevice::close()
{
    sdStop((SerialDriver*)sdef.serial);
    return true;
}

void UARTDevice::flush()
{
    //TODO: Handle this for UART ports
}

#ifndef HAL_UART_NODMA
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
/*
  DMA transmit completion interrupt handler
 */
void UARTDevice::tx_complete(void* self, uint32_t flags)
{
    UARTDevice* uart_dev = (UARTDevice*)self;
    chSysLockFromISR();

    // check nothing bad happened
    if ((flags & STM32_DMA_ISR_TEIF) != 0) {
        INTERNAL_ERROR(AP_InternalError::error_t::dma_fail);
    }

    dmaStreamDisable(uart_dev->txdma);
    uart_dev->_last_write_completed_us = AP_HAL::micros();

    chEvtSignalI(uart_dev->uart_thread_ctx, UARTDriver::EVT_TRANSMIT_DMA_COMPLETE);
    chSysUnlockFromISR();
}

/*
  write out pending bytes with DMA
 */
void UARTDevice::write_pending_bytes_DMA(uint32_t n)
{
    osalDbgAssert(!dma_handle || !dma_handle->is_locked(), "DMA handle is already locked");
    // sanity check
    if (!dma_handle) {
        return;
    }

    while (n > 0) {
        if (_flow_control != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE &&
            acts_line != 0 &&
            palReadLine(acts_line)) {
            // we are using hw flow control and the CTS line is high. We
            // will hold off trying to transmit until the CTS line goes
            // low to indicate the receiver has space. We do this before
            // we take the DMA lock to prevent a high CTS line holding a
            // DMA channel that may be needed by another device
            return;
        }

        uint16_t tx_len = 0;

        {
            WITH_SEMAPHORE(_write_mutex);
            // get some more to write
            tx_len = _writebuf.peekbytes(tx_bounce_buf, MIN(n, TX_BOUNCE_BUFSIZE));

            if (tx_len == 0) {
                return; // all done
            }
            // find out how much is still left to write while we still have the lock
            n = MIN(_writebuf.available(), n);
        }

        dma_handle->lock(); // we have our own thread so grab the lock

        chEvtGetAndClearEvents(UARTDriver::EVT_TRANSMIT_DMA_COMPLETE);

        if (dma_handle->has_contention()) {
            // on boards with a hw fifo we can use a higher threshold for disabling DMA
            if (_baudrate <= CONTENTION_BAUD_THRESHOLD) {
                contention_counter += 3;
                if (contention_counter > 1000) {
                    // more than 25% of attempts to use this DMA
                    // channel are getting contention and we have a
                    // low baudrate. Switch off DMA for future
                    // transmits on this low baudrate UART
                    tx_dma_enabled = false;
                    dma_handle->unlock(false);
                    break;
                }
            }
            /*
            someone else is using this same DMA channel. To reduce
            latency we will drop the TX size with DMA on this UART to
            keep TX times below 250us. This can still suffer from long
            times due to CTS blockage
            */
            uint32_t max_tx_bytes = 1 + (_baudrate * 250UL / 1000000UL);
            if (tx_len > max_tx_bytes) {
                tx_len = max_tx_bytes;
            }
        } else if (contention_counter > 0) {
            contention_counter--;
        }

        chSysLock();
        dmaStreamDisable(txdma);
        stm32_cacheBufferFlush(tx_bounce_buf, tx_len);
        dmaStreamSetMemory0(txdma, tx_bounce_buf);
        dmaStreamSetTransactionSize(txdma, tx_len);
        uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
        dmamode |= STM32_DMA_CR_CHSEL(sdef.dma_tx_channel_id);
        dmamode |= STM32_DMA_CR_PL(0);
#if defined(STM32H7)
        dmamode |= 1<<20;   // TRBUFF See 2.3.1 in the H743 errata
#endif
        dmaStreamSetMode(txdma, dmamode | STM32_DMA_CR_DIR_M2P |
                        STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
        dmaStreamEnable(txdma);
        uint32_t timeout_us = ((1000000UL * (tx_len+2) * 10) / _baudrate) + 500;
        chSysUnlock();
        // wait for the completion or timeout handlers to signal that we are done
        eventmask_t mask = chEvtWaitAnyTimeout(UARTDriver::EVT_TRANSMIT_DMA_COMPLETE, chTimeUS2I(timeout_us));
        // handle a TX timeout. This can happen with using hardware flow
        // control if CTS pin blocks transmit or sometimes the DMA completion simply disappears
        if (mask == 0) {
            chSysLock();
            // check whether DMA completion happened in the intervening time
            // first disable the stream to prevent further interrupts
            dmaStreamDisable(txdma);

            const uint32_t tx_size = dmaStreamGetTransactionSize(txdma);

            if (tx_size >= tx_len) {
                // we didn't write any of our bytes
                tx_len = 0;
            } else {
                // record how much was sent tx_size is how much was
                // not sent (could be 0)
                tx_len -= tx_size;
            }
            if (tx_len > 0) {
                _last_write_completed_us = AP_HAL::micros();
            }
            chEvtGetAndClearEvents(UARTDriver::EVT_TRANSMIT_DMA_COMPLETE);
            chSysUnlock();
        }
        // clean up pending locks
        dma_handle->unlock(mask & UARTDriver::EVT_TRANSMIT_DMA_COMPLETE);

        if (tx_len) {
            WITH_SEMAPHORE(_write_mutex);
            // skip over amount actually written
            _writebuf.advance(tx_len);

            // update stats
            _total_written += tx_len;
            _tx_stats_bytes += tx_len;

            n -= tx_len;
        } else {
            // if we didn't manage to transmit any bytes then stop
            // processing so we can check flow control state in outer
            // loop
            break;
        }
    }
}
#pragma GCC diagnostic pop
#endif // HAL_UART_NODMA

/*
  write any pending bytes to the device, non-DMA method
 */
void UARTDevice::write_pending_bytes_NODMA(uint32_t n)
{
    WITH_SEMAPHORE(_write_mutex);

    ByteBuffer::IoVec vec[2];
    uint16_t nwritten = 0;

    if (half_duplex && n > 1) {
        half_duplex_setup_tx();
    }

    const auto n_vec = _writebuf.peekiovec(vec, n);
    for (int i = 0; i < n_vec; i++) {
        int ret = -1;
        if (sdef.is_usb) {
            ret = 0;
#ifdef HAVE_USB_SERIAL
            ret = chnWriteTimeout((SerialUSBDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        } else {
#if HAL_USE_SERIAL == TRUE
            ret = chnWriteTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);
#endif
        }
        if (ret < 0) {
            break;
        }
        if (ret > 0) {
            _last_write_completed_us = AP_HAL::micros();
            nwritten += ret;
        }
        _writebuf.advance(ret);

        /* We wrote less than we asked for, stop */
        if ((unsigned)ret != vec[i].len) {
            break;
        }
    }

    _total_written += nwritten;
    _tx_stats_bytes += nwritten;
}

/*
  write any pending bytes to the device
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wframe-larger-than=128"
void UARTDevice::write()
{
    uint32_t now = AP_HAL::micros();
    bool need_tick = false;
    if (now - _last_write_call_us >= 1000) {
        // run the timer tick if it's been more than 1ms since we last run
        need_tick = true;
        _last_write_call_us = now;
    }

    eventmask_t mask = chEvtGetEventsX();
    if (!(((mask & UARTDriver::EVT_TRANSMIT_DATA_READY) || need_tick || (hd_tx_active && (mask & UARTDriver::EVT_TRANSMIT_END))))) {
        return;
    }

    // this sets _in_write_call to true and back to false on function exit
    AutoFlagResetter<bool> write_call(_in_write_call, true, false);

    // half duplex we do reads in the write thread
    if (half_duplex) {
        AutoFlagResetter<bool> read_call(_in_read_call, true, false);
        read_bytes_NODMA();
        if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
            chEvtSignal(_wait.thread_ctx, UARTDriver::EVT_DATA);
        }
    }

    if (hd_tx_active) {
        hd_tx_active &= ~chEvtGetAndClearFlags(&hd_listener);
        if (!hd_tx_active) {
            /*
                half-duplex transmit has finished. We now re-enable the
                HDSEL bit for receive
            */
            SerialDriver *sd = (SerialDriver*)(sdef.serial);
            sdStop(sd);
            sercfg.cr3 |= USART_CR3_HDSEL;
            sdStart(sd, &sercfg);
        }
    }

    // write any pending bytes
    uint32_t n = _writebuf.available();
    if (n <= 0) {
        return;
    }

#ifndef HAL_UART_NODMA
    if (tx_dma_enabled) {
        write_pending_bytes_DMA(n);
    } else
#endif
    {
        write_pending_bytes_NODMA(n);
    }

    // handle AUTO flow control mode
    if (_flow_control == AP_HAL::UARTDriver::FLOW_CONTROL_AUTO) {
        if (_first_write_started_us == 0) {
            _first_write_started_us = AP_HAL::micros();
            _total_written = 0;
        }
#ifndef HAL_UART_NODMA
        if (tx_dma_enabled) {
            // when we are using DMA we have a reliable indication that a write
            // has completed from the DMA completion interrupt
            if (_last_write_completed_us != 0) {
                _flow_control = AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE;
                return;
            }
        } else
#endif
        {
            // without DMA we need to look at the number of bytes written into the queue versus the
            // remaining queue space
            uint32_t space = qSpaceI(&((SerialDriver*)sdef.serial)->oqueue);
            uint32_t used = SERIAL_BUFFERS_SIZE - space;
            // threshold is 8 for the GCS_Common code to unstick SiK radios, which
            // sends 6 bytes with flow control disabled
            const uint8_t threshold = 8;
            if (_total_written > used && _total_written - used > threshold) {
                _flow_control = AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE;
                return;
            }
        }
        if (AP_HAL::micros() - _first_write_started_us > 500*1000UL) {
            // it doesn't look like hw flow control is working
            hal.console->printf("disabling flow control on serial %u\n", sdef.get_index());
            set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        }
    }

}
#pragma GCC diagnostic pop

/*
  setup for half duplex tramsmit. To cope with uarts that have level
  shifters and pullups we need to play a trick where we temporarily
  disable half-duplex while transmitting. That disables the receive
  part of the uart on the pin which allows the transmit side to
  correctly setup the idle voltage before the transmit starts.
 */
void UARTDevice::half_duplex_setup_tx(void)
{
    if (!hd_tx_active) {
        chEvtGetAndClearFlags(&hd_listener);
        // half-duplex transmission is done when both the output is empty and the transmission is ended
        // if we only wait for empty output the line can be setup for receive too soon losing data bits
        hd_tx_active = CHN_TRANSMISSION_END | CHN_OUTPUT_EMPTY;
        SerialDriver *sd = (SerialDriver*)(sdef.serial);
        sdStop(sd);
        sercfg.cr3 &= ~USART_CR3_HDSEL;
        sdStart(sd, &sercfg);
    }
}

void UARTDevice::read()
{
    if (half_duplex) {
        // half duplex we do reads in the write thread
        return;
    }
    AutoFlagResetter<bool> read_call(_in_read_call, true, false);
#ifndef HAL_UART_NODMA
    if (rx_dma_enabled && rxdma) {
        chSysLock();
        //Check if DMA is enabled
        //if not, it might be because the DMA interrupt was silenced
        //let's handle that here so that we can continue receiving
#if defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
        bool enabled = (rxdma->channel->CCR & STM32_DMA_CR_EN);
#else
        bool enabled = (rxdma->stream->CR & STM32_DMA_CR_EN);
#endif
        if (!enabled) {
            uint8_t len = RX_BOUNCE_BUFSIZE - dmaStreamGetTransactionSize(rxdma);
            if (len != 0) {
                _readbuf.write(rx_bounce_buf[rx_bounce_idx], len);
                _rx_stats_bytes += len;

                receive_timestamp_update();
                if (_rts_is_active) {
                    update_rts_line();
                }
            }
            // DMA disabled by idle interrupt never got a chance to be handled
            // we will enable it here
            dmaStreamDisable(rxdma);
            dma_rx_enable();
        }
        chSysUnlock();
    }
#endif

#ifndef HAL_UART_NODMA
    if (!rx_dma_enabled)
#endif
    {
        read_bytes_NODMA();
    }
    if (_wait.thread_ctx && _readbuf.available() >= _wait.n) {
        chEvtSignal(_wait.thread_ctx, UARTDriver::EVT_DATA);
    }
}

// regular serial read
void UARTDevice::read_bytes_NODMA()
{
    // try to fill the read buffer
    ByteBuffer::IoVec vec[2];

    const auto n_vec = _readbuf.reserve(vec, _readbuf.space());
    for (int i = 0; i < n_vec; i++) {
        int ret = 0;
        //Do a non-blocking read
        ret = chnReadTimeout((SerialDriver*)sdef.serial, vec[i].data, vec[i].len, TIME_IMMEDIATE);

        if (ret < 0) {
            break;
        }
#if CH_CFG_USE_EVENTS == TRUE
        if (parity_enabled && ((chEvtGetAndClearFlags(&ev_listener) & SD_PARITY_ERROR))) {
            // discard bytes with parity error
            ret = -1;
        }
#endif
        if (!hd_tx_active) {
            _readbuf.commit((unsigned)ret);
            _rx_stats_bytes += ret;
            receive_timestamp_update();
        }

        /* stop reading as we read less than we asked for */
        if ((unsigned)ret < vec[i].len) {
            break;
        }
    }
}

/*
  change flow control mode for port
 */
void UARTDevice::set_flow_control(AP_HAL::UARTDriver::flow_control flowcontrol)
{
    SerialDriver *sd = (SerialDriver*)(sdef.serial);
    _flow_control = (arts_line == 0) ? AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE : flowcontrol;

    switch (_flow_control) {

    case AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE:
        // force RTS active when flow disabled
        if (arts_line != 0) {
            palSetLineMode(arts_line, 1);
            palClearLine(arts_line);
        }
        _rts_is_active = true;
        // disable hardware CTS support
        chSysLock();
        if ((sd->usart->CR3 & (USART_CR3_CTSE | USART_CR3_RTSE)) != 0) {
            sd->usart->CR1 &= ~USART_CR1_UE;
            sd->usart->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
            sd->usart->CR1 |= USART_CR1_UE;
        }
        chSysUnlock();
        break;

    case AP_HAL::UARTDriver::FLOW_CONTROL_AUTO:
        // reset flow control auto state machine
        _first_write_started_us = 0;
        _last_write_completed_us = 0;
        FALLTHROUGH;

    case AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE:
        // we do RTS in software as STM32 hardware RTS support toggles
        // the pin for every byte which loses a lot of bandwidth
        palSetLineMode(arts_line, 1);
        palClearLine(arts_line);
        _rts_is_active = true;
        // enable hardware CTS support, disable RTS support as we do that in software
        chSysLock();
        if ((sd->usart->CR3 & (USART_CR3_CTSE | USART_CR3_RTSE)) != USART_CR3_CTSE) {
            // CTSE and RTSE can only be written when uart is disabled
            sd->usart->CR1 &= ~USART_CR1_UE;
            sd->usart->CR3 |= USART_CR3_CTSE;
            sd->usart->CR3 &= ~USART_CR3_RTSE;
            sd->usart->CR1 |= USART_CR1_UE;
        }
        chSysUnlock();
        break;
    }
}

/*
  software update of rts line. We don't use the HW support for RTS as
  it has no hysteresis, so it ends up toggling RTS on every byte
 */
void UARTDevice::update_rts_line(void)
{
    if (arts_line == 0 || _flow_control == AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE) {
        return;
    }
    uint16_t space = _readbuf.space();
    if (_rts_is_active && space < 16) {
        _rts_is_active = false;
        palSetLine(arts_line);
    } else if (!_rts_is_active && space > 32) {
        _rts_is_active = true;
        palClearLine(arts_line);
    }
}

/*
  setup parity
 */
void UARTDevice::configure_parity(uint8_t v)
{
    // stop and start to take effect
    sdStop((SerialDriver*)sdef.serial);

#ifdef USART_CR1_M0
    // cope with F3 and F7 where there are 2 bits in CR1_M
    const uint32_t cr1_m0 = USART_CR1_M0;
#else
    const uint32_t cr1_m0 = USART_CR1_M;
#endif

    switch (v) {
    case 0:
        // no parity
        sercfg.cr1 &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
        break;
    case 1:
        // odd parity
        // setting USART_CR1_M ensures extra bit is used as parity
        // not last bit of data
        sercfg.cr1 |= cr1_m0 | USART_CR1_PCE;
        sercfg.cr1 |= USART_CR1_PS;
        break;
    case 2:
        // even parity
        sercfg.cr1 |= cr1_m0 | USART_CR1_PCE;
        sercfg.cr1 &= ~USART_CR1_PS;
        break;
    }

    sdStart((SerialDriver*)sdef.serial, &sercfg);

#if CH_CFG_USE_EVENTS == TRUE
    if (parity_enabled) {
        chEvtUnregister(chnGetEventSource((SerialDriver*)sdef.serial), &ev_listener);
    }
    parity_enabled = (v != 0);
    if (parity_enabled) {
        chEvtRegisterMaskWithFlags(chnGetEventSource((SerialDriver*)sdef.serial),
                                   &ev_listener,
                                   UARTDriver::EVT_PARITY,
                                   SD_PARITY_ERROR);
    }
#endif

#ifndef HAL_UART_NODMA
    if (rx_dma_enabled) {
        // Configure serial driver to skip handling RX packets
        // because we will handle them via DMA
        ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
    }
#endif
}

/*
  set stop bits
 */
void UARTDevice::set_stop_bits(uint8_t n)
{
    // stop and start to take effect
    sdStop((SerialDriver*)sdef.serial);

    switch (n) {
    case 1:
        _cr2_options &= ~USART_CR2_STOP2_BITS;
        _cr2_options |= USART_CR2_STOP1_BITS;
        break;
    case 2:
        _cr2_options &= ~USART_CR2_STOP1_BITS;
        _cr2_options |= USART_CR2_STOP2_BITS;
        break;
    }
    sercfg.cr2 = _cr2_options;

    sdStart((SerialDriver*)sdef.serial, &sercfg);
#ifndef HAL_UART_NODMA
    if (rx_dma_enabled) {
        //Configure serial driver to skip handling RX packets
        //because we will handle them via DMA
        ((SerialDriver*)sdef.serial)->usart->CR1 &= ~USART_CR1_RXNEIE;
    }
#endif
}

/*
 set user specified PULLUP/PULLDOWN options from SERIALn_OPTIONS
*/
void UARTDevice::set_pushpull(uint16_t options)
{
    if ((options & AP_HAL::UARTDriver::OPTION_PULLDOWN_RX) && arx_line) {
        palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLDOWN);
    }
    if ((options & AP_HAL::UARTDriver::OPTION_PULLDOWN_TX) && atx_line) {
        palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLDOWN);
    }
    if ((options & AP_HAL::UARTDriver::OPTION_PULLUP_RX) && arx_line) {
        palLineSetPushPull(arx_line, PAL_PUSHPULL_PULLUP);
    }
    if ((options & AP_HAL::UARTDriver::OPTION_PULLUP_TX) && atx_line) {
        palLineSetPushPull(atx_line, PAL_PUSHPULL_PULLUP);
    }
}

// set optional features, return true on success
bool UARTDevice::set_options(uint16_t options)
{
    bool ret = true;

    _last_options = options;

    SerialDriver *sd = (SerialDriver*)(sdef.serial);
    uint32_t cr2 = sd->usart->CR2;
    uint32_t cr3 = sd->usart->CR3;
    bool was_enabled = (sd->usart->CR1 & USART_CR1_UE);

    /*
      allow for RX, TX, RTS and CTS pins to be remapped via BRD_ALT_CONFIG
     */
    arx_line = GPIO::resolve_alt_config(sdef.rx_line, PERIPH_TYPE::UART_RX, sdef.instance);
    atx_line = GPIO::resolve_alt_config(sdef.tx_line, PERIPH_TYPE::UART_TX, sdef.instance);
    arts_line = GPIO::resolve_alt_config(sdef.rts_line, PERIPH_TYPE::OTHER, sdef.instance);
    acts_line = GPIO::resolve_alt_config(sdef.cts_line, PERIPH_TYPE::OTHER, sdef.instance);

    // Check flow control, might have to disable if RTS line is gone
    set_flow_control(_flow_control);

#if defined(STM32F7) || defined(STM32H7) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
    // F7 has built-in support for inversion in all uarts
    ioline_t rx_line = (options & AP_HAL::UARTDriver::OPTION_SWAP)?atx_line:arx_line;
    ioline_t tx_line = (options & AP_HAL::UARTDriver::OPTION_SWAP)?arx_line:atx_line;

    // if we are half-duplex then treat either inversion option as
    // both being enabled. This is easier to understand for users, who
    // can be confused as to which pin is the one that needs inversion
    if ((options & AP_HAL::UARTDriver::OPTION_HDPLEX) && (options & (AP_HAL::UARTDriver::OPTION_TXINV|AP_HAL::UARTDriver::OPTION_RXINV)) != 0) {
        options |= AP_HAL::UARTDriver::OPTION_TXINV|AP_HAL::UARTDriver::OPTION_RXINV;
    }

    if (options & AP_HAL::UARTDriver::OPTION_RXINV) {
        cr2 |= USART_CR2_RXINV;
        _cr2_options |= USART_CR2_RXINV;
        if (rx_line != 0) {
            palLineSetPushPull(rx_line, PAL_PUSHPULL_PULLDOWN);
        }
    } else {
        cr2 &= ~USART_CR2_RXINV;
        _cr2_options &= ~USART_CR2_RXINV;
        if (rx_line != 0) {
            palLineSetPushPull(rx_line, PAL_PUSHPULL_PULLUP);
        }
    }
    if (options & AP_HAL::UARTDriver::OPTION_TXINV) {
        cr2 |= USART_CR2_TXINV;
        _cr2_options |= USART_CR2_TXINV;
        if (tx_line != 0) {
            palLineSetPushPull(tx_line, PAL_PUSHPULL_PULLDOWN);
        }
    } else {
        cr2 &= ~USART_CR2_TXINV;
        _cr2_options &= ~USART_CR2_TXINV;
        if (tx_line != 0) {
            palLineSetPushPull(tx_line, PAL_PUSHPULL_PULLUP);
        }
    }
    // F7 can also support swapping RX and TX pins
    if (options & AP_HAL::UARTDriver::OPTION_SWAP) {
        cr2 |= USART_CR2_SWAP;
        _cr2_options |= USART_CR2_SWAP;
    } else {
        cr2 &= ~USART_CR2_SWAP;
        _cr2_options &= ~USART_CR2_SWAP;
    }
#else // STM32F4
    // F4 can do inversion by GPIO if enabled in hwdef.dat, using
    // TXINV and RXINV options
    if (options & AP_HAL::UARTDriver::OPTION_RXINV) {
        if (sdef.rxinv_gpio >= 0) {
            hal.gpio->write(sdef.rxinv_gpio, sdef.rxinv_polarity);
        } else {
            ret = false;
        }
    }
    if (options & AP_HAL::UARTDriver::OPTION_TXINV) {
        if (sdef.txinv_gpio >= 0) {
            hal.gpio->write(sdef.txinv_gpio, sdef.txinv_polarity);
        } else {
            ret = false;
        }
    }
    if (options & AP_HAL::UARTDriver::OPTION_SWAP) {
        ret = false;
    }
#endif // STM32xx

    // both F4 and F7 can do half-duplex
    if (options & AP_HAL::UARTDriver::OPTION_HDPLEX) {
        cr3 |= USART_CR3_HDSEL;
        _cr3_options |= USART_CR3_HDSEL;
        if (!half_duplex) {
            chEvtRegisterMaskWithFlags(chnGetEventSource((SerialDriver*)sdef.serial),
                                       &hd_listener,
                                       UARTDriver::EVT_TRANSMIT_END,
                                       CHN_OUTPUT_EMPTY | CHN_TRANSMISSION_END);
            half_duplex = true;
        }
#ifndef HAL_UART_NODMA
        if (rx_dma_enabled && rxdma) {
            dmaStreamDisable(rxdma);
        }
#endif
        // force DMA off when using half-duplex as the timing may affect other devices
        // sharing the DMA channel
        rx_dma_enabled = tx_dma_enabled = false;
    } else {
        cr3 &= ~USART_CR3_HDSEL;
        _cr3_options &= ~USART_CR3_HDSEL;
    }

    set_pushpull(options);

    if (sd->usart->CR2 == cr2 &&
        sd->usart->CR3 == cr3) {
        // no change
        return ret;
    }

    if (was_enabled) {
        sd->usart->CR1 &= ~USART_CR1_UE;
    }

    sd->usart->CR2 = cr2;
    sd->usart->CR3 = cr3;

    if (was_enabled) {
        sd->usart->CR1 |= USART_CR1_UE;
    }
    return ret;
}

// get optional features
uint16_t UARTDevice::get_options(void) const
{
    return _last_options;
}

// record timestamp of new incoming data
void UARTDevice::receive_timestamp_update(void)
{
    _receive_timestamp[_receive_timestamp_idx^1] = AP_HAL::micros64();
    _receive_timestamp_idx ^= 1;
}

// request information on uart I/O for @SYS/uarts.txt for this uart
void UARTDevice::get_info(ExpandingString &str)
{
    uint32_t now_ms = AP_HAL::millis();
    str.printf("UART%u ", unsigned(sdef.instance));
    str.printf("TX%c=%8u RX%c=%8u TXBD=%6u RXBD=%6u\n",
               tx_dma_enabled ? '*' : ' ',
               unsigned(_tx_stats_bytes),
               rx_dma_enabled ? '*' : ' ',
               unsigned(_rx_stats_bytes),
               unsigned(_tx_stats_bytes * 10000 / (now_ms - _last_stats_ms)),
               unsigned(_rx_stats_bytes * 10000 / (now_ms - _last_stats_ms)));
    _tx_stats_bytes = 0;
    _rx_stats_bytes = 0;
    _last_stats_ms = now_ms;
}

/*
  software control of the CTS pin if available. Return false if
  not available
*/
bool UARTDevice::set_CTS_pin(bool high)
{
    if (_flow_control != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE) {
        // CTS pin is being used
        return false;
    }
    if (acts_line == 0) {
        // we don't have a CTS pin on this UART
        return false;
    }
    palSetLineMode(acts_line, 1);
    palWriteLine(acts_line, high?1:0);
    return true;
}

/*
  software control of the RTS pin if available. Return false if
  not available
*/
bool UARTDevice::set_RTS_pin(bool high)
{
    if (_flow_control != AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE) {
        // RTS pin is being used
        return false;
    }
    if (arts_line == 0) {
        // we don't have a RTS pin on this UART
        return false;
    }
    palSetLineMode(arts_line, 1);
    palWriteLine(arts_line, high?1:0);
    return true;
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
