/* mbed Microcontroller Library
 * Copyright (c) 2016 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MBED_OBJECTS_H
#define MBED_OBJECTS_H

#include "cmsis.h"
#include "PortNames.h"
#include "PeripheralNames.h"
#include "PinNames.h"
#include "stdbool.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    IRQ_NOT_SET,
    IRQ_ON,
    IRQ_OFF
} irq_setting_t;

struct port_s {
    __IO uint32_t *reg_dir;
    __IO uint32_t *reg_out;
    __IO uint32_t *reg_val;
    __IO uint32_t *reg_drv;
    PortName port;
    uint32_t mask;
};

// Check if needed in KM
struct gpio_irq_s {
    /* Don't bother with having a port number here as there's only one */
    uint32_t ch;   /* Corresponds to the interrupt pin */
};

struct serial_s {     
    volatile struct uart_s *reg_base;
    PinName rx_pin;
    PinName tx_pin;
    uint8_t index; /* IRQ index number. Might get removed later, unsure at this stage */
    irq_setting_t irq_rx_setting; /* used in serial_irq_set if IRQ has been set or not. Might get removed later, unsure at this stage */
    irq_setting_t irq_tx_setting;		
};

struct i2c_s {
    volatile struct i2c_ss *reg_base;
    PinName i2c_sda_pin;
    PinName i2c_scl_pin;
};

struct spi_s {
    volatile struct spi_ss *reg_base;
    uint32_t timeout;
};

#include "gpio_object.h"

#ifdef __cplusplus
}
#endif

#endif
