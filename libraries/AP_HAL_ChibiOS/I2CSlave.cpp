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
 */
#include <AP_HAL/AP_HAL.h>

#ifdef HAL_I2C_SLAVE_LIST
#include "I2CSlave.h"
#include "ch.h"
#include "hal.h"

// values calculated with STM32CubeMX tool, PCLK=54MHz
#define HAL_I2C_F7_100_TIMINGR 0x20404768
#define HAL_I2C_F7_400_TIMINGR 0x6000030D

#define HAL_I2C_H7_100_TIMINGR 0x00707CBB
#define HAL_I2C_H7_400_TIMINGR 0x00300F38

static struct I2CSlaveInfo {
    I2C_TypeDef *i2c;
    ioline_t scl_line;
    ioline_t sda_line;
    uint32_t EV_IRQn;
    ChibiOS::I2CSlave* slv;
} I2CD[] = { HAL_I2C_SLAVE_LIST };

void ChibiOS::I2CSlave::init(uint8_t i2c_bus, uint8_t slave_address, uint32_t busclock, ReceiveCb rx_cb, TransmitCb tx_cb)
{
    if (I2CD[i2c_bus].slv) {
        AP_HAL::panic("Initialising same bus twice\n");
    }
    I2C_TypeDef *i2c = I2CD[i2c_bus].i2c;
    I2CD[i2c_bus].slv = this;
    _i2c_bus = i2c_bus;
    // Set Callbacks
    _rx_cb = rx_cb;
    _tx_cb = tx_cb;


#if STM32_HAS_I2C1
    if (I2C1 == i2c) {
        rccResetI2C1();
        rccEnableI2C1(true);
    }
#endif

#if STM32_HAS_I2C2
    if (I2C2 == i2c) {
        rccResetI2C2();
        rccEnableI2C2(true);
    }
#endif

#if STM32_HAS_I2C3
    if (I2C3 == i2c) {
        rccResetI2C3();
        rccEnableI2C3(true);
    }
#endif

#if STM32_HAS_I2C4
    if (I2C4 == i2c) {
        rccResetI2C4();
        rccEnableI2C4(true);
    }
#endif

    //Disable I2C
    i2c->CR1 &= ~I2C_CR1_PE;

    //Enable Analog Filter
    i2c->CR1 &= ~I2C_CR1_ANFOFF;

    //Disable Digital Filter
    i2c->CR1 &=  ~(I2C_CR1_DNF);

#if defined(STM32F7) || defined(STM32F3)
    if (busclock <= 100000) {
        i2c->TIMINGR = HAL_I2C_F7_100_TIMINGR;
    } else {
        i2c->TIMINGR = HAL_I2C_F7_400_TIMINGR;
    }
#elif defined(STM32H7)
    if (busclock <= 100000) {
        i2c->TIMINGR = HAL_I2C_H7_100_TIMINGR;
    } else {
        i2c->TIMINGR = HAL_I2C_H7_400_TIMINGR;
    }
#else // F1 or F4
        // NOTE: TODO
        // if (businfo[i].i2ccfg.clock_speed <= 100000) {
        //     businfo[i].i2ccfg.duty_cycle = STD_DUTY_CYCLE;
        // } else {
        //     businfo[i].i2ccfg.duty_cycle = FAST_DUTY_CYCLE_2;
        // }
#endif

    //Enable Stretching
    i2c->CR1 &= ~I2C_CR1_NOSTRETCH;
    i2c->CR1 |= I2C_CR1_SMBDEN;

    //7Bit Address Mode
    i2c->CR2 &= ~I2C_CR2_ADD10;

    i2c->OAR1 = (slave_address & 0x7F) << 1; //I2C Slave Address
    i2c->OAR1 |= (1<<15);

    //Enable I2C interrupt
    nvicEnableVector(I2CD[i2c_bus].EV_IRQn, 3);

    i2c->CR1 |= I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_ADDRIE;
    i2c->CR1 |= I2C_CR1_PE; // Enable I2C
}

void ChibiOS::I2CSlave::isr_handler_trampoline(I2C_TypeDef *i2c)
{
    uint8_t idx;
    for (idx = 0; idx < ARRAY_SIZE(I2CD); idx++) {
        if (I2CD[idx].i2c == i2c && I2CD[idx].slv) {
            I2CD[idx].slv->isr_handler();
        }
    }

}

void ChibiOS::I2CSlave::isr_handler()
{
    if (_i2c_bus == -1) {
        return;
    }

    uint32_t isr = I2CD[_i2c_bus].i2c->ISR;
    if (isr & (1<<3)) { // ADDR
        _i2c_transfer_address = (isr >> 17) & 0x7FU; // ADDCODE
        _i2c_transfer_direction = (isr >> 16) & 1; // direction
        _i2c_transfer_byte_idx = 0;
        if (_i2c_transfer_direction) {
            I2CD[_i2c_bus].i2c->ISR |= (1<<0); // TXE
        }
        I2CD[_i2c_bus].i2c->ICR |= (1<<3); // ADDRCF
    }

    if (isr & I2C_ISR_RXNE) {
        uint8_t recv_byte = I2CD[_i2c_bus].i2c->RXDR & 0xff; // reading clears our interrupt flag
        _rx_cb(_i2c_transfer_byte_idx, recv_byte);
        _i2c_transfer_byte_idx++;
    }

    if (isr & I2C_ISR_TXIS) {
        uint8_t send_byte = 0;
        send_byte = _tx_cb();
        I2CD[_i2c_bus].i2c->TXDR = send_byte;

        _i2c_transfer_byte_idx++;
    }
}


#if defined(STM32_I2C1_EVENT_HANDLER) && !STM32_I2C_USE_I2C1
CH_IRQ_HANDLER(STM32_I2C1_EVENT_HANDLER);
CH_IRQ_HANDLER(STM32_I2C1_EVENT_HANDLER) {
    CH_IRQ_PROLOGUE();
    ChibiOS::I2CSlave::isr_handler_trampoline(I2C1);
    CH_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_I2C2_EVENT_HANDLER) && !STM32_I2C_USE_I2C2
CH_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER);
CH_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER) {
    CH_IRQ_PROLOGUE();
    ChibiOS::I2CSlave::isr_handler_trampoline(I2C2);
    CH_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_I2C3_EVENT_HANDLER) && !STM32_I2C_USE_I2C3
CH_IRQ_HANDLER(STM32_I2C3_EVENT_HANDLER);
CH_IRQ_HANDLER(STM32_I2C3_EVENT_HANDLER) {
    CH_IRQ_PROLOGUE();
    ChibiOS::I2CSlave::isr_handler_trampoline(I2C3);
    CH_IRQ_EPILOGUE();
}
#endif

#if defined(STM32_I2C4_EVENT_HANDLER) && !STM32_I2C_USE_I2C4
CH_IRQ_HANDLER(STM32_I2C4_EVENT_HANDLER);
CH_IRQ_HANDLER(STM32_I2C4_EVENT_HANDLER) {
    CH_IRQ_PROLOGUE();
    ChibiOS::I2CSlave::isr_handler_trampoline(I2C4);
    CH_IRQ_EPILOGUE();
}
#endif

#endif //#ifdef HAL_I2C_SLAVE_LIST
