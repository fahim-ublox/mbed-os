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

#define MAX_FREQ    26000000UL
#define MIN_FREQ    13000000UL
#define PerCLK   26000000UL // Peripheral clock: 75Mhz
#define PCLK    PerCLK // APB clock

/** Macro generating the mask for a bitfield of \p n bits */
#define DRIVER_BIT_SET(n)                      (1u<<(n))

#define DRIVER_BITS_CLR(data, mask)            ((data) & ~(mask))
#define DRIVER_BITS_SET(data, bits)            ((data) |  (bits))

/** Macro generating the mask for a bitfield of \p n bits */
#define DRIVER_BIT_MASK(n)                      (DRIVER_BIT_SET(n) - 1)

/** Macro generating the mask for a bitfield defined as `name_OFFSET` and `name_SIZE` */
#define DRIVER_BITFIELD_MASK_(name)             (DRIVER_BIT_MASK(name##_SIZE) << (name##_OFFSET))
#define DRIVER_BITFIELD_MASK(name)              DRIVER_BITFIELD_MASK_(name)

/** Extract bitfield defined as `name_OFFSET` and `name_SIZE` from \p data */
#define DRIVER_BITFIELD_GET_(data, name)        (((data) >> name##_OFFSET) & DRIVER_BIT_MASK(name##_SIZE))
#define DRIVER_BITFIELD_GET(data, name)         DRIVER_BITFIELD_GET_(data, name)

/** Return \p data with bitfield defined as `name_OFFSET` and `name_SIZE` cleared */
#define DRIVER_BITFIELD_CLR(data, name)         ((data) & ~DRIVER_BITFIELD_MASK(name))

/** Return \p bitfield defined as `name_OFFSET` and `name_SIZE` set to \p value */
#define DRIVER_BITFIELD_VAL_(name, value)       (((value) & DRIVER_BIT_MASK(name##_SIZE)) << name##_OFFSET)
#define DRIVER_BITFIELD_VAL(name, value)        DRIVER_BITFIELD_VAL_(name, value)

/** Return \p data with bitfield defined as `name_OFFSET` and `name_SIZE` set to \p value */
#define DRIVER_BITFIELD_SET(data, name, value)  (DRIVER_BITFIELD_CLR(data, name) | DRIVER_BITFIELD_VAL(name, value))

/** Return \p bitfield defined as `name_OFFSET` and `name_SIZE` set to \p name_ENUM_value */
#define DRIVER_BITFIELD_ENUM_(name, enumValue)      DRIVER_BITFIELD_VAL(name, name##_ENUM_##enumValue)
#define DRIVER_BITFIELD_ENUM(name, enumValue)       DRIVER_BITFIELD_ENUM_(name, enumValue)

/** Return \p data with bitfield defined as `name_OFFSET` and `name_SIZE` set to \p name_ENUM_value */
#define DRIVER_BITFIELD_SET_ENUM(data, name, enumValue)  DRIVER_BITFIELD_SET(data, name,  DRIVER_BITFIELD_ENUM(name, enumValue))

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
    /*int pioChannel = 0;
    PioPeriphMux mux;
	PinName pinArray[4] = {mosi, miso, sclk, ssel};

    MBED_ASSERT(mosi != (PinName)NC && miso != (PinName)NC && sclk != (PinName)NC && ssel != (PinName)NC);

	if(mosi == SPI1_MOSI && miso == SPI1_MISO) {
        obj->reg_base = SPI1_BASE;
	} else {
        obj->reg_base = SPI2_BASE;
	}

    obj->reg_base->crc = DRIVER_BITFIELD_MASK(SPI_CR_CR_TXDMA) | DRIVER_BITFIELD_MASK(SPI_CR_CR_RXDMA) |
                          DRIVER_BITFIELD_MASK(SPI_CR_CR_ENABLE);

	for(int i=0;i<4;i++) {
		pioChannel = gpio_channel_select(pinArray[i],PinChannelMap);
		MBED_ASSERT(pioChannel != ERROR);

	    mux = pio_periph_muxing_get(pinArray[i], PinPeripheralMap);
		MBED_ASSERT(mux != PIO_MUX_FALSE);

		status = gpio_periph_mux_set(mux,pioChannel,false);
		MBED_ASSERT(status == SUCCESS);
	}*/
	obj->reg_base = SPI2_BASE;
    obj->reg_base->crc = DRIVER_BITFIELD_MASK(SPI_CR_CR_TXDMA) | DRIVER_BITFIELD_MASK(SPI_CR_CR_RXDMA) |
                          DRIVER_BITFIELD_MASK(SPI_CR_CR_ENABLE);
	//obj->reg_base->cr |= SPI_CR_CR_LOCLB_ENABLE_VALUE << SPI_CR_CR_LOCLB_OFFSET; // 5: Local loop back mode (shift register output connected to input in master mode)
	//obj->reg_base->cr |= SPI_CR_CR_LOCKS_ENABLE_ACTIVE_ONLY_DURING_MASTER_MODE_VALUE << SPI_CR_CR_LOCKS_OFFSET; // 9: Keep CS asserted in master mode also when TxFifo is empty
	//obj->reg_base->cr |= SPI_CR_CR_CONTINUESCK_ENABLE_ACTIVE_ONLY_DURING_MASTER_MODE_VALUE << SPI_CR_CR_CONTINUESCK_OFFSET; // 10: Keep on generating SCK in master mode also when TxFifo is empty
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
    if(slave == 0) {
        obj->reg_base->cr |= SPI_CR_CR_MASTER_MASTER_MODE_VALUE << SPI_CR_CR_MASTER_OFFSET; // Master Mode Selected
    } else {
        obj->reg_base->cr |= SPI_CR_CR_MASTER_SLAVE_MODE_VALUE << SPI_CR_CR_MASTER_OFFSET; // Slave Mode Selected
    }
	
	// only 8-bit word length supported, 9-bit mode (1 status bit + 8 data bits) to implement flow control
    MBED_ASSERT(bits == 8 || bits == 9);
	
    if(bits == 8) {
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
    uint32_t dataSize;
	
    if(bit_status(obj->reg_base->cr, SPI_CR_CR_MASTER_OFFSET)) {
        MBED_ASSERT(hz <= MAX_FREQ); // The maximum bus clock speed in Master mode shall be 26 MHz
    } else {
        MBED_ASSERT(hz >= MIN_FREQ); // Bus-clock speed in Slave mode shall not be less than 13 MHz.
    }

    hz=162000; /* TODO: the low clock frequency is required for FPGA operation, will remove this line and return to 1 MHz operation on the real chip */
    clockDiv = (PCLK / (2 * hz)) - 1;
    if(clockDiv < 0xFF) {
        obj->reg_base->cdr = clockDiv; // Clock divider register: SCLK period = 2*(<ClockDiv>+1)/PCLK
    } else {
        MBED_ASSERT("Invalid Clock Divider");
    }

    obj->reg_base->tor = 0xFF; // Timeout counter register: timeout = 8*(<ClockDiv>+1)*<timeout>/ PCLK 

    p_waitDiv = PCLK / 1000 * 6 / (clockDiv + 1) / 1000; // 6us wait
    if(p_waitDiv > 0 && p_waitDiv < 0xFF) {
        obj->reg_base->wsr = p_waitDiv - 1; 
        obj->reg_base->wgr = p_waitDiv - 1; 
    } else {
        MBED_ASSERT("Invalid Wait Divider");
    }
	obj->reg_base->csr = DRIVER_BITFIELD_SET(obj->reg_base->csr, SPI_CSR_NONE, 0);		
    //dataSize = bit_status(obj->reg_base->cr, SPI_CR_CR_FLOWCTRL_OFFSET) == SPI_CR_CR_MASTER_MASTER_MODE_VALUE ? 8 : 9;
    //obj->timeout = ((8 * 2 + (obj->reg_base->wgr + 1)) * dataSize + (obj->reg_base->wsr + 1)) * (obj->reg_base->cdr + 1)/(MAX_FREQ / 1000); //duration in us
	obj->reg_base->cr |= SPI_CR_CR_ENABLE_ENABLE_VALUE << SPI_CR_CR_ENABLE_OFFSET; // 0: Enable SPI
}

/*****************************************************************************
                                spi_master_write:
                Write a byte out in master mode and receive a value
*****************************************************************************/
int spi_master_write(spi_t *obj, int value)
{	
	obj->reg_base->thr = (uint8_t)value;
    while(!(bit_status(obj->reg_base->sr, SPI_SR_SR_TXEMPTY_OFFSET))); // Wait for TX buffer empty

    //obj->reg_base->crc = SPI_CR_CR_ENABLE_ENABLE_VALUE << SPI_CR_CR_ENABLE_OFFSET;
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
    return bit_status(obj->reg_base->sr, SPI_SR_SR_RXNOEMPTY_OFFSET) == 1 ? 1 : 0;
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
    while(!(bit_status(obj->reg_base->sr, SPI_SR_SR_TXEMPTY_OFFSET))); // Wait for TX buffer empty
}
	
/*****************************************************************************
                               spi_busy:
           Checks if the specified SPI peripheral is in use
*****************************************************************************/
int spi_busy(spi_t *obj)
{
    return obj->reg_base->twcr == 0 ? 0 : 1;
}

