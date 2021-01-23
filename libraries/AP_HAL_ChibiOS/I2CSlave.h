/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
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
 * Modified for use in AP_HAL_ChibiOS by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once
#include "AP_HAL_ChibiOS.h"

#ifdef HAL_I2C_SLAVE_LIST
namespace ChibiOS {
class I2CSlave {
public:
    FUNCTOR_TYPEDEF(ReceiveCb, void, uint8_t, uint8_t);
    FUNCTOR_TYPEDEF(TransmitCb, uint8_t);
    void init(uint8_t i2c_bus, uint8_t slave_address, uint32_t busclock, ReceiveCb rx_cb, TransmitCb tx_cb);
    static void isr_handler_trampoline(I2C_TypeDef *i2c);
private:
    void isr_handler();
    
    uint8_t _i2c_transfer_byte_idx;
    uint8_t _i2c_transfer_address;
    uint8_t _i2c_transfer_direction;
    int8_t _i2c_bus = -1;

    ChibiOS::I2CSlave::ReceiveCb _rx_cb;
    ChibiOS::I2CSlave::TransmitCb _tx_cb;
};

}
#endif
