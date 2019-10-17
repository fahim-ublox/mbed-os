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

#include "mbed_assert.h"
#include "spi_api.h"
#include "KM_app.h"
#include "objects.h"
#include "pinmap.h"


/** SPI base address */
#define SPI1_BASE    app_ss_app_spi1
#define SPI2_BASE    app_ss_app_spi2

#define PerCLK   5200000UL // Peripheral clock
#define PCLK    PerCLK // APB clock


static inline uint32_t shift(uint32_t x)
{
    return (0x1u << x);
}

static inline uint32_t bit_status(uint32_t x, uint32_t y)
{
    return (x & shift(y)) == 0 ? 0 : 1;
}

/*****************************************************************************
                                spi_init:
                           Initialize the SPI

*****************************************************************************/
void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    uint32_t status;
    int pioChannel = 0;
    PioPeriphMux mux;
    PinName pinArray[4] = {mosi, miso, sclk, ssel};

    MBED_ASSERT(mosi != (PinName)NC && miso != (PinName)NC && sclk != (PinName)NC /*&& ssel != (PinName)NC*/);

    for (int i=0; i<3; i++) {
    	pioChannel = gpio_channel_select(pinArray[i],PinChannelMap);
    	MBED_ASSERT(pioChannel != ERROR);

    	mux = pio_periph_muxing_get(pinArray[i], PinPeripheralMap);
    	MBED_ASSERT(mux != PIO_MUX_FALSE);

    	status = gpio_periph_mux_set(mux,pioChannel,false);
    	MBED_ASSERT(status == SUCCESS);
    }

    if (mosi == SPI1_MOSI && miso == SPI1_MISO) {
    	obj->reg_base = SPI1_BASE;
    } else {
    	obj->reg_base = SPI2_BASE;
    }
}

/*****************************************************************************
                                spi_free:
                          Release a SPI object
*****************************************************************************/
void spi_free(spi_t *obj)
{
	obj->reg_base->cr |= SPI_CR_CR_ENABLE_DISABLE_VALUE << SPI_CR_CR_ENABLE_OFFSET; // Disable SPI 
}
	
/*****************************************************************************
                                spi_format:
                          Configure the SPI format
*****************************************************************************/
void spi_format(spi_t *obj, int bits, int mode, int slave)
{
    if (slave == 0) {
        obj->reg_base->cr |= SPI_CR_CR_MASTER_MASTER_MODE_VALUE << SPI_CR_CR_MASTER_OFFSET; // Master Mode Selected
    } else {
        obj->reg_base->cr |= SPI_CR_CR_MASTER_SLAVE_MODE_VALUE << SPI_CR_CR_MASTER_OFFSET; // Slave Mode Selected
    }
	
	// only 8 and 9 bit word length supported, 9-bit mode (1 status bit + 8 data bits) to implement flow control
    MBED_ASSERT(bits == 8 || bits == 9);
	
    if (bits == 8) {
        obj->reg_base->cr |= SPI_CR_CR_FLOWCTRL_NORMAL_OPERATION_VALUE << SPI_CR_CR_FLOWCTRL_OFFSET;
    } else {
        obj->reg_base->cr |= SPI_CR_CR_FLOWCTRL_FLOW_CONTROL_9_BIT_MODE_ENABLED_VALUE << SPI_CR_CR_FLOWCTRL_OFFSET;
    }
		
    switch (mode) {
        case 0:
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPOL_SCLK_LOW_WHEN_IDLE_VALUE << SPI_CR_CR_CLOCKPOL_OFFSET;
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPHA_FIRST_DATA_VALID_ON_FIRST_SCLK_EDGE_VALUE << SPI_CR_CR_CLOCKPHA_OFFSET;
            break;
        case 1:
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPOL_SCLK_LOW_WHEN_IDLE_VALUE << SPI_CR_CR_CLOCKPOL_OFFSET;
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPHA_FIRST_DATA_VALID_ON_SECOND_SCLK_EDGE_VALUE << SPI_CR_CR_CLOCKPHA_OFFSET;
            break;
        case 2:
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPOL_SCLK_HIGH_WHEN_IDLE_VALUE << SPI_CR_CR_CLOCKPOL_OFFSET;
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPHA_FIRST_DATA_VALID_ON_FIRST_SCLK_EDGE_VALUE << SPI_CR_CR_CLOCKPHA_OFFSET;
            break;
        case 3:
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPOL_SCLK_HIGH_WHEN_IDLE_VALUE << SPI_CR_CR_CLOCKPOL_OFFSET;
            obj->reg_base->cr |= SPI_CR_CR_CLOCKPHA_FIRST_DATA_VALID_ON_SECOND_SCLK_EDGE_VALUE << SPI_CR_CR_CLOCKPHA_OFFSET;
            break;
        default:
            break;
    }
}

/*****************************************************************************
                                spi_frequency:
                            Set the SPI baud rate
*****************************************************************************/
void spi_frequency(spi_t *obj, int hz)
{
    uint8_t clockDiv = 0, p_waitDiv = 0;
	
    if (bit_status(obj->reg_base->cr, SPI_CR_CR_MASTER_OFFSET)) {
        MBED_ASSERT(hz <= PerCLK/2); // The maximum bus clock speed in Master mode shall be SCK <= PerCLKxCI/2
    } else {
        MBED_ASSERT(hz <= PerCLK/6); // The maximum bus clock speed in Slave mode shall be SCK <= PerCLKxCI/6
    }

    clockDiv = (PCLK / (2 * hz)) - 1;
    if (clockDiv < 0xFF) {
        obj->reg_base->cdr = clockDiv; // Clock divider register: SCLK period = 2*(<ClockDiv>+1)/PCLK
    } else {
        MBED_ASSERT(clockDiv < 0xFF); // Invalid Clock Divider
    }

    obj->reg_base->tor = 0xFF; // Timeout counter register: timeout = 8*(<ClockDiv>+1)*<timeout>/ PCLK 

    p_waitDiv = PCLK / 1000 * 6 / (clockDiv + 1) / 1000; // 6us wait
    if (p_waitDiv > 0 && p_waitDiv < 0xFF) {
        obj->reg_base->wsr = p_waitDiv - 1; 
        obj->reg_base->wgr = p_waitDiv - 1; 
    } else {
        MBED_ASSERT(p_waitDiv > 0 && p_waitDiv < 0xFF); // Invalid Wait Divider
    }

    obj->reg_base->cr |= SPI_CR_CR_ENABLE_ENABLE_VALUE << SPI_CR_CR_ENABLE_OFFSET; // 0: Enable SPI
}

/*****************************************************************************
                                spi_master_write:
                Write a byte out in master mode and receive a value
*****************************************************************************/
int spi_master_write(spi_t *obj, int value)
{	
    obj->reg_base->thr = (uint8_t)value;
    while (!(bit_status(obj->reg_base->sr, SPI_SR_SR_TXEMPTY_OFFSET))); // Wait for TX buffer empty

    return  obj->reg_base->rhr;
}

/*****************************************************************************
                               spi_master_block_write:
                  Write a block out in master mode and receive a value
*****************************************************************************/
int spi_master_block_write(spi_t *obj, const char *tx_buffer, int tx_length, char *rx_buffer, int rx_length, char write_fill)
{
    char in,out;
    int total = (tx_length > rx_length) ? tx_length : rx_length;
	
    for (int i = 0; i < total; i++) {
        out = (i < tx_length) ? tx_buffer[i] : write_fill;
        in = spi_master_write(obj, out);
        if (i < rx_length) {
            rx_buffer[i] = in;
        }
    }
    return total;
}

/*****************************************************************************
                               spi_slave_receive:
                    Check if a value is available to read
*****************************************************************************/
int spi_slave_receive(spi_t *obj)
{
    return bit_status(obj->reg_base->sr, SPI_SR_SR_RXNOTEMPTY_OFFSET) == 1 ? 1 : 0;
}

/*****************************************************************************
                               spi_slave_read:
     Get a received value out of the SPI receive buffer in slave mode
*****************************************************************************/
int spi_slave_read(spi_t *obj)
{
    while (!spi_slave_receive(obj)) {
        if (!obj->timeout--) {
            return 0;
        }
    }
    return obj->reg_base->rhr;		
}

/*****************************************************************************
                               spi_slave_write:
              Write a value to the SPI peripheral in slave mode
*****************************************************************************/
void spi_slave_write(spi_t *obj, int value)
{
    obj->reg_base->thr = value;
    while (!(bit_status(obj->reg_base->sr, SPI_SR_SR_TXEMPTY_OFFSET))); // Wait for TX buffer empty
}
	
/*****************************************************************************
                               spi_busy:
           Checks if the specified SPI peripheral is in use
*****************************************************************************/
int spi_busy(spi_t *obj)
{
    return obj->reg_base->twcr == 0 ? 0 : 1;
}

