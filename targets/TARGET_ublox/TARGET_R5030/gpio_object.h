/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
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
#ifndef MBED_GPIO_OBJECT_H
#define MBED_GPIO_OBJECT_H

#include "mbed_assert.h"
#include "KM_app.h"
#include "pio.h"


#ifdef __cplusplus
extern "C" {
#endif


// ---------------------------------------------------------------------------
// Macros
// ---------------------------------------------------------------------------
/** Extract bit corresponding to \p channel from \p data */
#define PIO_CHANNEL_BIT_GET(register, channel)       (((register) >> channel) & 0x1u)

/** KM can support up to 64 PIO channels */
#define MAX_NUM_PIO_CHANNELS     64

/** Each register is 32 bits which can have 32 PIO channels */
#define PIO_CHANNEL_SUB_32_MASK     0x1F

/** Right shift this value to get the value for multiple of 32, which can be used to calculate the PIO register base for a specific Channel */
#define PIO_CHANNEL_OVER_32_SHIFT     5

/** GPIO base address */
#define gpio_base     app_ss_pio_onfi_wrap_maps_pio

#define ERROR    -1
#define SUCCESS   1
#define FALSE     0
#define TRUE      1


typedef struct {
    PinName  pin;
    uint32_t mask;
    __IO struct pio_s *reg_base;
} gpio_t;

typedef struct {
    PinName  pin;
    __IO int channel;
} PinToChannel;

typedef struct {
    PinName  pin;
    __IO int peripheral;
} PinToPeripheral;


typedef enum {
    PIO_MUX_FALSE,
    PIO_MUX_GPIO,
    PIO_MUX_PERIPH_0,
    PIO_MUX_PERIPH_1,
    PIO_MUX_PERIPH_2,
    PIO_MUX_PERIPH_3,
    PIO_MUX_LAST
} PioPeriphMux;

static const PinToChannel PinChannelMap[] = {
    {LED0, 0},
    {LED1, 1},  
    {LED2, 2},
    {LED3, 3},
    {LED4, 4},
    {LED5, 5},
    {LED6, 6},
    {UART1_RX, 42},
    {UART1_TX, 43},
    {UART1_CTS, 44},
    {UART1_RTS, 45},
    {UART2_RX, 30},
    {UART2_TX, 31},
    {I2C_SCL, 50},
    {I2C_SDA, 51},
    {SPI1_MOSI, 3},
    {SPI1_MISO, 4},
    {SPI1_CLK, 5},
    {SPI1_CS, 6},
    {SPI2_MOSI, 46},
    {SPI2_MISO, 47},
    {SPI2_CLK, 48},
    {SPI2_CS, 49},
    {NC, NC}
};

static const PinToPeripheral PinPeripheralMap[] = {
    {LED0, PIO_MUX_GPIO},
    {LED1, PIO_MUX_GPIO},  
    {LED2, PIO_MUX_GPIO},
    {LED3, PIO_MUX_GPIO},
    {LED4, PIO_MUX_GPIO},
    {LED5, PIO_MUX_GPIO},
    {LED6, PIO_MUX_GPIO},
    {UART1_RX, PIO_MUX_PERIPH_0},
    {UART1_TX, PIO_MUX_PERIPH_0},
    {UART1_CTS, PIO_MUX_PERIPH_0},
    {UART1_RTS, PIO_MUX_PERIPH_0},
    {UART2_RX, PIO_MUX_PERIPH_2},
    {UART2_TX, PIO_MUX_PERIPH_2},
    {I2C_SCL, PIO_MUX_PERIPH_0},
    {I2C_SDA, PIO_MUX_PERIPH_0},
    {SPI1_MOSI, PIO_MUX_PERIPH_1},
    {SPI1_MISO, PIO_MUX_PERIPH_1},
    {SPI1_CLK, PIO_MUX_PERIPH_1},
    {SPI1_CS, PIO_MUX_PERIPH_1},
    {SPI2_MOSI, PIO_MUX_PERIPH_1},
    {SPI2_MISO, PIO_MUX_PERIPH_1},
    {SPI2_CLK, PIO_MUX_PERIPH_1},
    {SPI2_CS, PIO_MUX_PERIPH_1},
    {NC, PIO_MUX_FALSE}
};

/*****************************************************************************
                                gpio_write:
                            Write data on gpio
*****************************************************************************/
static inline void gpio_write(gpio_t *obj, int value)
{
    MBED_ASSERT(obj->pin != (PinName)NC);

    if (value) {
    	obj->reg_base->pio_sodr_0 = obj->mask;  // Set output pin
    } else {
    	obj->reg_base->pio_codr_0 = obj->mask; // Clear output pin
    }
}

/*****************************************************************************
                                gpio_read:
                            Read data on gpio
*****************************************************************************/
static inline int gpio_read(gpio_t *obj)
{
    MBED_ASSERT(obj->pin != (PinName)NC);
    return ((obj->reg_base->pio_pdsr_0 & obj->mask) ? 1 : 0);
}

/*****************************************************************************
                              gpio_is_connected:
                      Checks if gpio object is connected
*****************************************************************************/
static inline int gpio_is_connected(const gpio_t *obj)
{
    return obj->pin != (PinName)NC;
}

int gpio_channel_select(PinName pin, const PinToChannel* map);
PioPeriphMux pio_periph_muxing_get(PinName pin, const PinToPeripheral* map);
int gpio_periph_mux_set(PioPeriphMux mux, uint8_t pioChannel, bool periphPullUpDownOn);

#ifdef __cplusplus
}
#endif

#endif
