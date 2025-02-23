/*
 * Copyright (c) 2023, Raspberry Pi Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions, and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions, and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the copyright holder nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "util.h"
#include "gpio.h"



/* RP1 gpio chip */
#define GPIO_CHIP_NAME "rp1"
#define GPIO_COMPATIBLE "raspberrypi,rp1-gpio"
#define GPIO_CHIP_SIZE 0x30000
#define RP1_NUM_GPIOS 54

/* safe to use gpio numbers */
#define IS_HEADER_PIN(number) ((number) >= 2 && (number) <= 27)

/*
 * There are three banks and all the interested gpio's are in the second one
 * gpio pins 0-27 (2-27 are in the 40 pin header, and 0, 1 are reserved for advanced use) are in io_bank0
 */
static const int RP1_BANK_BASE[] = {0, 28, 34}; // how many pins in each bank {*, 0, 1}

/*
 * 1. write value to the (base + peri_offset + reg_offset) register/memory address
 * 2. read value from the register/memory address at (base + peri_offset + reg_offset)
 */
#define _write32(base, peri_offset, reg_offset, value) \
base[(peri_offset + reg_offset)/4] = value

#define _read32(base, peri_offset, reg_offset) \
base[(peri_offset + reg_offset)/4]

/*
 **** rio(registered io) interface allows us to manipulate gpio through host processor. <- rp1 datasheet 3.3.
 */
#define RP1_IO_BANK0_OFFSET      0x00000000
#define RP1_IO_BANK1_OFFSET      0x00004000
#define RP1_IO_BANK2_OFFSET      0x00008000
#define RP1_SYS_RIO_BANK0_OFFSET 0x00010000
#define RP1_SYS_RIO_BANK1_OFFSET 0x00014000
#define RP1_SYS_RIO_BANK2_OFFSET 0x00018000
#define RP1_PADS_BANK0_OFFSET    0x00020000
#define RP1_PADS_BANK1_OFFSET    0x00024000
#define RP1_PADS_BANK2_OFFSET    0x00028000

#define RP1_RW_OFFSET  0x0000
#define RP1_XOR_OFFSET 0x1000
#define RP1_SET_OFFSET 0x2000
#define RP1_CLR_OFFSET 0x3000

#define RP1_GPIO_CTRL_FSEL_LSB     0
#define RP1_GPIO_CTRL_FSEL_MASK    (0x1f << RP1_GPIO_CTRL_FSEL_LSB)
#define RP1_GPIO_CTRL_OUTOVER_LSB  12
#define RP1_GPIO_CTRL_OUTOVER_MASK (0x03 << RP1_GPIO_CTRL_OUTOVER_LSB)
#define RP1_GPIO_CTRL_OEOVER_LSB   14
#define RP1_GPIO_CTRL_OEOVER_MASK  (0x03 << RP1_GPIO_CTRL_OEOVER_LSB)

#define RP1_PADS_OD_SET       (1 << 7)
#define RP1_PADS_IE_SET       (1 << 6)
#define RP1_PADS_PUE_SET      (1 << 3)
#define RP1_PADS_PDE_SET      (1 << 2)

#define RP1_GPIO_IO_REG_STATUS_OFFSET(offset) (((offset * 2) + 0) * sizeof(uint32_t))
#define RP1_GPIO_IO_REG_CTRL_OFFSET(offset)   (((offset * 2) + 1) * sizeof(uint32_t))
#define RP1_GPIO_PADS_REG_OFFSET(offset)      (sizeof(uint32_t) + (offset * sizeof(uint32_t)))

#define RP1_GPIO_SYS_RIO_REG_OUT_OFFSET        0x0
#define RP1_GPIO_SYS_RIO_REG_OE_OFFSET         0x4
#define RP1_GPIO_SYS_RIO_REG_SYNC_IN_OFFSET    0x8


/* offsets in a struct of convenience. */
typedef struct {
    uint32_t io[3];
    uint32_t pads[3];
    uint32_t sys_rio[3];
} OFFSETS;

static const OFFSETS offsets = {
    .io = {RP1_IO_BANK0_OFFSET, RP1_IO_BANK1_OFFSET, RP1_IO_BANK2_OFFSET},
    .pads = {RP1_PADS_BANK0_OFFSET, RP1_PADS_BANK1_OFFSET, RP1_PADS_BANK2_OFFSET},
    .sys_rio = {RP1_SYS_RIO_BANK0_OFFSET, RP1_SYS_RIO_BANK1_OFFSET, RP1_SYS_RIO_BANK2_OFFSET},
};

/* function selection... */
typedef enum {
    RP1_FSEL_ALT0       = 0x0,
    RP1_FSEL_ALT1       = 0x1,
    RP1_FSEL_ALT2       = 0x2,
    RP1_FSEL_ALT3       = 0x3,
    RP1_FSEL_ALT4       = 0x4,
    RP1_FSEL_ALT5       = 0x5,
    RP1_FSEL_ALT6       = 0x6,
    RP1_FSEL_ALT7       = 0x7,
    RP1_FSEL_ALT8       = 0x8,
    RP1_FSEL_COUNT,
    RP1_FSEL_SYS_RIO    = RP1_FSEL_ALT5,
    RP1_FSEL_NULL       = 0x1f
} FN_IDX;


/*
 * Functions could be allocated with each GPIO.
 * For pin 0-27: https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf
 */

static const char *rp1_gpio_fsel_names[RP1_NUM_GPIOS][RP1_FSEL_COUNT] __attribute__((unused))=
{
    { "SPI0_SIO3" , "DPI_PCLK"     , "TXD1"         , "SDA0"         , 0              , "SYS_RIO00" , "PROC_RIO00" , "PIO0"       , "SPI2_CE0" , },
    { "SPI0_SIO2" , "DPI_DE"       , "RXD1"         , "SCL0"         , 0              , "SYS_RIO01" , "PROC_RIO01" , "PIO1"       , "SPI2_SIO1", },
    { "SPI0_CE3"  , "DPI_VSYNC"    , "CTS1"         , "SDA1"         , "IR_RX0"       , "SYS_RIO02" , "PROC_RIO02" , "PIO2"       , "SPI2_SIO0", },
    { "SPI0_CE2"  , "DPI_HSYNC"    , "RTS1"         , "SCL1"         , "IR_TX0"       , "SYS_RIO03" , "PROC_RIO03" , "PIO3"       , "SPI2_SCLK", },
    { "GPCLK0"    , "DPI_D0"       , "TXD2"         , "SDA2"         , "RI0"          , "SYS_RIO04" , "PROC_RIO04" , "PIO4"       , "SPI3_CE0" , },
    { "GPCLK1"    , "DPI_D1"       , "RXD2"         , "SCL2"         , "DTR0"         , "SYS_RIO05" , "PROC_RIO05" , "PIO5"       , "SPI3_SIO1", },
    { "GPCLK2"    , "DPI_D2"       , "CTS2"         , "SDA3"         , "DCD0"         , "SYS_RIO06" , "PROC_RIO06" , "PIO6"       , "SPI3_SIO0", },
    { "SPI0_CE1"  , "DPI_D3"       , "RTS2"         , "SCL3"         , "DSR0"         , "SYS_RIO07" , "PROC_RIO07" , "PIO7"       , "SPI3_SCLK", },
    { "SPI0_CE0"  , "DPI_D4"       , "TXD3"         , "SDA0"         , 0              , "SYS_RIO08" , "PROC_RIO08" , "PIO8"       , "SPI4_CE0" , },
    { "SPI0_MISO" , "DPI_D5"       , "RXD3"         , "SCL0"         , 0              , "SYS_RIO09" , "PROC_RIO09" , "PIO9"       , "SPI4_SIO0", },
    { "SPI0_MOSI" , "DPI_D6"       , "CTS3"         , "SDA1"         , 0              , "SYS_RIO010", "PROC_RIO010", "PIO10"      , "SPI4_SIO1", },
    { "SPI0_SCLK" , "DPI_D7"       , "RTS3"         , "SCL1"         , 0              , "SYS_RIO011", "PROC_RIO011", "PIO11"      , "SPI4_SCLK", },
    { "PWM0_CHAN0", "DPI_D8"       , "TXD4"         , "SDA2"         , "AAUD_LEFT"    , "SYS_RIO012", "PROC_RIO012", "PIO12"      , "SPI5_CE0" , },
    { "PWM0_CHAN1", "DPI_D9"       , "RXD4"         , "SCL2"         , "AAUD_RIGHT"   , "SYS_RIO013", "PROC_RIO013", "PIO13"      , "SPI5_SIO1", },
    { "PWM0_CHAN2", "DPI_D10"      , "CTS4"         , "SDA3"         , "TXD0"         , "SYS_RIO014", "PROC_RIO014", "PIO14"      , "SPI5_SIO0", },
    { "PWM0_CHAN3", "DPI_D11"      , "RTS4"         , "SCL3"         , "RXD0"         , "SYS_RIO015", "PROC_RIO015", "PIO15"      , "SPI5_SCLK", },
    { "SPI1_CE2"  , "DPI_D12"      , "DSI0_TE_EXT"  , 0              , "CTS0"         , "SYS_RIO016", "PROC_RIO016", "PIO16"      , },
    { "SPI1_CE1"  , "DPI_D13"      , "DSI1_TE_EXT"  , 0              , "RTS0"         , "SYS_RIO017", "PROC_RIO017", "PIO17"      , },
    { "SPI1_CE0"  , "DPI_D14"      , "I2S0_SCLK"    , "PWM0_CHAN2"   , "I2S1_SCLK"    , "SYS_RIO018", "PROC_RIO018", "PIO18"      , "GPCLK1",   },
    { "SPI1_MISO" , "DPI_D15"      , "I2S0_WS"      , "PWM0_CHAN3"   , "I2S1_WS"      , "SYS_RIO019", "PROC_RIO019", "PIO19"      , },
    { "SPI1_MOSI" , "DPI_D16"      , "I2S0_SDI0"    , "GPCLK0"       , "I2S1_SDI0"    , "SYS_RIO020", "PROC_RIO020", "PIO20"      , },
    { "SPI1_SCLK" , "DPI_D17"      , "I2S0_SDO0"    , "GPCLK1"       , "I2S1_SDO0"    , "SYS_RIO021", "PROC_RIO021", "PIO21"      , },
    { "SD0_CLK"   , "DPI_D18"      , "I2S0_SDI1"    , "SDA3"         , "I2S1_SDI1"    , "SYS_RIO022", "PROC_RIO022", "PIO22"      , },
    { "SD0_CMD"   , "DPI_D19"      , "I2S0_SDO1"    , "SCL3"         , "I2S1_SDO1"    , "SYS_RIO023", "PROC_RIO023", "PIO23"      , },
    { "SD0_DAT0"  , "DPI_D20"      , "I2S0_SDI2"    , 0              , "I2S1_SDI2"    , "SYS_RIO024", "PROC_RIO024", "PIO24"      , "SPI2_CE1" , },
    { "SD0_DAT1"  , "DPI_D21"      , "I2S0_SDO2"    , "MIC_CLK"      , "I2S1_SDO2"    , "SYS_RIO025", "PROC_RIO025", "PIO25"      , "SPI3_CE1" , },
    { "SD0_DAT2"  , "DPI_D22"      , "I2S0_SDI3"    , "MIC_DAT0"     , "I2S1_SDI3"    , "SYS_RIO026", "PROC_RIO026", "PIO26"      , "SPI5_CE1" , },
    { "SD0_DAT3"  , "DPI_D23"      , "I2S0_SDO3"    , "MIC_DAT1"     , "I2S1_SDO3"    , "SYS_RIO027", "PROC_RIO027", "PIO27"      , "SPI1_CE1" , },
    { "SD1_CLK"   , "SDA4"         , "I2S2_SCLK"    , "SPI6_MISO"    , "VBUS_EN0"     , "SYS_RIO10" , "PROC_RIO10" , },
    { "SD1_CMD"   , "SCL4"         , "I2S2_WS"      , "SPI6_MOSI"    , "VBUS_OC0"     , "SYS_RIO11" , "PROC_RIO11" , },
    { "SD1_DAT0"  , "SDA5"         , "I2S2_SDI0"    , "SPI6_SCLK"    , "TXD5"         , "SYS_RIO12" , "PROC_RIO12" , },
    { "SD1_DAT1"  , "SCL5"         , "I2S2_SDO0"    , "SPI6_CE0"     , "RXD5"         , "SYS_RIO13" , "PROC_RIO13" , },
    { "SD1_DAT2"  , "GPCLK3"       , "I2S2_SDI1"    , "SPI6_CE1"     , "CTS5"         , "SYS_RIO14" , "PROC_RIO14" , },
    { "SD1_DAT3"  , "GPCLK4"       , "I2S2_SDO1"    , "SPI6_CE2"     , "RTS5"         , "SYS_RIO15" , "PROC_RIO15" , },
    { "PWM1_CHAN2", "GPCLK3"       , "VBUS_EN0"     , "SDA4"         , "MIC_CLK"      , "SYS_RIO20" , "PROC_RIO20" , },
    { "SPI8_CE1"  , "PWM1_CHAN0"   , "VBUS_OC0"     , "SCL4"         , "MIC_DAT0"     , "SYS_RIO21" , "PROC_RIO21" , },
    { "SPI8_CE0"  , "TXD5"         , "PCIE_CLKREQ_N", "SDA5"         , "MIC_DAT1"     , "SYS_RIO22" , "PROC_RIO22" , },
    { "SPI8_MISO" , "RXD5"         , "MIC_CLK"      , "SCL5"         , "PCIE_CLKREQ_N", "SYS_RIO23" , "PROC_RIO23" , },
    { "SPI8_MOSI" , "RTS5"         , "MIC_DAT0"     , "SDA6"         , "AAUD_LEFT"    , "SYS_RIO24" , "PROC_RIO24" , "DSI0_TE_EXT", },
    { "SPI8_SCLK" , "CTS5"         , "MIC_DAT1"     , "SCL6"         , "AAUD_RIGHT"   , "SYS_RIO25" , "PROC_RIO25" , "DSI1_TE_EXT", },
    { "PWM1_CHAN1", "TXD5"         , "SDA4"         , "SPI6_MISO"    , "AAUD_LEFT"    , "SYS_RIO26" , "PROC_RIO26" , },
    { "PWM1_CHAN2", "RXD5"         , "SCL4"         , "SPI6_MOSI"    , "AAUD_RIGHT"   , "SYS_RIO27" , "PROC_RIO27" , },
    { "GPCLK5"    , "RTS5"         , "VBUS_EN1"     , "SPI6_SCLK"    , "I2S2_SCLK"    , "SYS_RIO28" , "PROC_RIO28" , },
    { "GPCLK4"    , "CTS5"         , "VBUS_OC1"     , "SPI6_CE0"     , "I2S2_WS"      , "SYS_RIO29" , "PROC_RIO29" , },
    { "GPCLK5"    , "SDA5"         , "PWM1_CHAN0"   , "SPI6_CE1"     , "I2S2_SDI0"    , "SYS_RIO210", "PROC_RIO210", },
    { "PWM1_CHAN3", "SCL5"         , "SPI7_CE0"     , "SPI6_CE2"     , "I2S2_SDO0"    , "SYS_RIO211", "PROC_RIO211", },
    { "GPCLK3"    , "SDA4"         , "SPI7_MOSI"    , "MIC_CLK"      , "I2S2_SDI1"    , "SYS_RIO212", "PROC_RIO212", "DSI0_TE_EXT", },
    { "GPCLK5"    , "SCL4"         , "SPI7_MISO"    , "MIC_DAT0"     , "I2S2_SDO1"    , "SYS_RIO213", "PROC_RIO213", "DSI1_TE_EXT", },
    { "PWM1_CHAN0", "PCIE_CLKREQ_N", "SPI7_SCLK"    , "MIC_DAT1"     , "TXD5"         , "SYS_RIO214", "PROC_RIO214", },
    { "SPI8_SCLK" , "SPI7_SCLK"    , "SDA5"         , "AAUD_LEFT"    , "RXD5"         , "SYS_RIO215", "PROC_RIO215", },
    { "SPI8_MISO" , "SPI7_MOSI"    , "SCL5"         , "AAUD_RIGHT"   , "VBUS_EN2"     , "SYS_RIO216", "PROC_RIO216", },
    { "SPI8_MOSI" , "SPI7_MISO"    , "SDA6"         , "AAUD_LEFT"    , "VBUS_OC2"     , "SYS_RIO217", "PROC_RIO217", },
    { "SPI8_CE0"  , 0              , "SCL6"         , "AAUD_RIGHT"   , "VBUS_EN3"     , "SYS_RIO218", "PROC_RIO218", },
    { "SPI8_CE1"  , "SPI7_CE0"     , 0              , "PCIE_CLKREQ_N", "VBUS_OC3"     , "SYS_RIO219", "PROC_RIO219", },
};

/* ------------------------------------------------------------------------------------------------------------------ */

/* Some helper functions to read/write registers. */
static uint32_t ctrl_read(volatile uint32_t *base, int bank, int offset){
    return _read32(base, offsets.io[bank], RP1_GPIO_IO_REG_CTRL_OFFSET(offset));
}

static void ctrl_write(volatile uint32_t *base, int bank, int offset, uint32_t value){
    _write32(base, offsets.io[bank], RP1_GPIO_IO_REG_CTRL_OFFSET(offset), value);
}

static uint32_t pads_read(volatile uint32_t *base, int bank, int offset){
    return _read32(base, offsets.pads[bank], RP1_GPIO_PADS_REG_OFFSET(offset));
}

static void pads_write(volatile uint32_t *base, int bank, int offset, uint32_t value){
    _write32(base, offsets.pads[bank], RP1_GPIO_PADS_REG_OFFSET(offset), value);
}

static uint32_t sys_rio_out_read(volatile uint32_t *base, int bank, int offset){
    UNUSED(offset);
    return _read32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_OUT_OFFSET);
}

static uint32_t sys_rio_sync_in_read(volatile uint32_t *base, int bank, int offset){
    UNUSED(offset);
    return _read32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_SYNC_IN_OFFSET);
}

static void sys_rio_out_set(volatile uint32_t *base, int bank, int offset){
    _write32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_OUT_OFFSET + RP1_SET_OFFSET, 1U << offset);
}

static void sys_rio_out_clr(volatile uint32_t *base, int bank, int offset){
    _write32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_OUT_OFFSET + RP1_CLR_OFFSET, 1U << offset);
}

static uint32_t sys_rio_oe_read(volatile uint32_t *base, int bank){
    return _read32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_OE_OFFSET);
}

static void sys_rio_oe_clr(volatile uint32_t *base, int bank, int offset){
    _write32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_OE_OFFSET + RP1_CLR_OFFSET, 1U << offset);
}

static void sys_rio_oe_set(volatile uint32_t *base, int bank, int offset){
    _write32(base, offsets.sys_rio[bank], RP1_GPIO_SYS_RIO_REG_OE_OFFSET + RP1_SET_OFFSET, 1U << offset);
}

/* Interfacing methods. */

/* @brief Set the GPIO mode: OUTPUT or INPUT. */
void set_direction(GPIO *gpio, DIRECTION dir){
    if (dir == DIR_INPUT)
        sys_rio_oe_clr(gpio->chip->base_address, gpio->bank, gpio->offset);
    else if (dir == DIR_OUTPUT)
        sys_rio_oe_set(gpio->chip->base_address, gpio->bank, gpio->offset);
    else
        assert(0);
}

/* @brief Retunrs the GPIO mode: OUTPUT or INPUT. */
DIRECTION get_direction(GPIO *gpio){
    uint32_t reg = sys_rio_oe_read(gpio->chip->base_address, gpio->bank);
    return (reg & (1U << gpio->offset)) ? DIR_OUTPUT : DIR_INPUT;
}

/* @brief Returns the function GPIO do. */
FUNCTION get_function(GPIO *gpio){
    uint32_t reg = ctrl_read(gpio->chip->base_address, gpio->bank, gpio->offset);
    FN_IDX rsel  = ((reg & RP1_GPIO_CTRL_FSEL_MASK) >> RP1_GPIO_CTRL_FSEL_LSB);
    FUNCTION fsel;
    if (rsel == RP1_FSEL_SYS_RIO)
        fsel = FN_GPIO;
    else if (rsel == RP1_FSEL_NULL)
        fsel = FN_NONE;
    else if (rsel < RP1_FSEL_COUNT)
        fsel = (FUNCTION)rsel;
    else
        fsel = FN_MAX;
    return fsel;
}

/* @brief Select the function in GPIO do: INPUT or OUTPUT and more. */
void set_function(GPIO *gpio, const FUNCTION func){
    uint32_t ctrl_reg;
    uint32_t pad_reg;
    uint32_t old_pad_reg;
    FN_IDX rsel;

    if (func < (FUNCTION)RP1_FSEL_COUNT)
        rsel = (FN_IDX)func;
    else if (func == FN_INPUT || func == FN_OUTPUT || func == FN_GPIO)
        rsel = RP1_FSEL_SYS_RIO;
    else if (func == FN_NONE)
        rsel = RP1_FSEL_NULL;
    else
        return;
    if (func == FN_INPUT)
        set_direction(gpio, DIR_INPUT);
    else if (func == FN_OUTPUT)
        set_direction(gpio, DIR_OUTPUT);

    ctrl_reg = ctrl_read(gpio->chip->base_address, gpio->bank, gpio->offset) & ~RP1_GPIO_CTRL_FSEL_MASK;
    ctrl_reg |= rsel << RP1_GPIO_CTRL_FSEL_LSB;
    ctrl_write(gpio->chip->base_address, gpio->bank, gpio->offset, ctrl_reg);

    pad_reg = pads_read(gpio->chip->base_address, gpio->bank, gpio->offset);
    old_pad_reg = pad_reg;

    if (rsel == RP1_FSEL_NULL){
        pad_reg &= ~RP1_PADS_IE_SET; // Disable input
    }else {
        pad_reg |= RP1_PADS_IE_SET; // Enable input
    }
    if (rsel != RP1_FSEL_NULL){
        pad_reg &= ~RP1_PADS_OD_SET; // Enable peripheral func output
    }else {
        pad_reg |= RP1_PADS_OD_SET; // Disable peripheral func output
    }
    if (pad_reg != old_pad_reg)
        pads_write(gpio->chip->base_address, gpio->bank, gpio->offset, pad_reg);
}

/* @brief Returns the level: 1 or 0 */
int get_level(GPIO *gpio){
    uint32_t pad_reg;
    uint32_t reg;

    pad_reg = pads_read(gpio->chip->base_address, gpio->bank, gpio->offset);
    if (!(pad_reg & RP1_PADS_IE_SET))
	return -1;
    reg = sys_rio_sync_in_read(gpio->chip->base_address, gpio->bank, gpio->offset);

    return (reg & (1U << gpio->offset)) ? 1 : 0;
}

/* @brief Set the logic value if set to OUTPUT: HIGH or LOW. */
void set_drive(GPIO *gpio, DRIVE drv) {
    if (drv == DRIVE_HIGH) {
        sys_rio_out_set(gpio->chip->base_address, gpio->bank, gpio->offset);
    }else if (drv == DRIVE_LOW) {
        sys_rio_out_clr(gpio->chip->base_address, gpio->bank, gpio->offset);
    }
}

/* @brief Pull the GPIO pin: UP or DOWN. */
void set_pull(GPIO *gpio, PULL pull){
    uint32_t reg = pads_read(gpio->chip->base_address, gpio->bank, gpio->offset);
    reg &= ~(RP1_PADS_PDE_SET | RP1_PADS_PUE_SET);
    if (pull == PULL_UP)
        reg |= RP1_PADS_PUE_SET;
    else if (pull == PULL_DOWN)
        reg |= RP1_PADS_PDE_SET;
    pads_write(gpio->chip->base_address, gpio->bank, gpio->offset, reg);
}

/* @brief Returns to where GPIO pin is pulled. */
PULL get_pull(GPIO *gpio){
    PULL pull = PULL_NONE;
    uint32_t reg = pads_read(gpio->chip->base_address, gpio->bank, gpio->offset);
    if (reg & RP1_PADS_PUE_SET)
        pull = PULL_UP;
    else if (reg & RP1_PADS_PDE_SET)
        pull = PULL_DOWN;

    return pull;
}

/* @brief Returns the logic value if set to OUTPUT: HIGH or LOW. */
DRIVE get_drive(GPIO *gpio){
    uint32_t reg = sys_rio_out_read(gpio->chip->base_address, gpio->bank, gpio->offset);
    return (reg & (1U << gpio->offset)) ? DRIVE_HIGH : DRIVE_LOW;
}

/* ------------------------------------------------------------------------------------------------------------------ */

void *memory_map(const GPIO_CHIP *chip) {
    void *map;

    int page_size = getpagesize();
    uint64_t physical_address = chip->physical_address;
    int size = chip->size;
    unsigned align = chip->physical_address & (page_size - 1);
    unsigned len = size + align; // map length
    int fd = chip->mem_fd; // file to map

    if (fd < 0) {
        fd = open("/dev/mem", O_RDWR|O_SYNC);
        if (fd < 0) {
            return NULL;
        } else {
            uint64_t offset = physical_address - align; // offset to GPIO peripheral
            UNUSED(offset); // to make compiler happy!
        }
    }
    uint64_t offset = 0; // offset to GPIO peripheral

    map = mmap(
                NULL,                          /* Any address in our space will do */
                len,                            /* size + align */
                PROT_READ | PROT_WRITE,   /* Enable reading & writing */
                MAP_SHARED,               /* Shared with other processes */
                fd,
                offset
                );

    if (map == MAP_FAILED) {
        return NULL;
    }

    return (void *)((char *)map + align);

}

void free_gpio(GPIO *gpio) {
    gpio->set_function(gpio, FN_NONE);
    free((void *)gpio);
}

void free_gpio_chip(GPIO_CHIP *chip) {
    free((void *)chip);
}

GPIO_CHIP *gpio_chip_init(void) {
    unsigned i = 0, gpiomem_idx = 0;

    GPIO_CHIP *chip = malloc(sizeof(GPIO_CHIP));

    chip->name = GPIO_CHIP_NAME;
    chip->size = GPIO_CHIP_SIZE;
    chip->num_gpios = RP1_NUM_GPIOS;

    /* We find some members of GPIO_CHIP from the device tree */

    const char *device_tree_path = "/proc/device-tree/";
    char path_buffer[FILENAME_MAX]; // a temp buffer to store search path along the device tree
    char *alias = NULL;

    dt_set_path(device_tree_path);
    sprintf(path_buffer, "gpio%d", i);
    alias = dt_read_prop("/aliases", path_buffer, NULL);
    chip->device_tree_node = alias;

    if (!alias) {
        alias = dt_read_prop("/aliases", "gpio", NULL);
    }
    if (!alias)
        return NULL;

    chip->compatible = GPIO_COMPATIBLE;

    if (dt_parse_addr(alias) == INVALID_ADDRESS) {
        dt_free(alias);
        return NULL;
    }

    chip->physical_address = dt_parse_addr(alias);
    sprintf(path_buffer, "/dev/gpiomem%d", gpiomem_idx);
    chip->mem_fd = open(path_buffer, O_RDWR|O_SYNC);
    chip->base_address = (volatile uint32_t *)memory_map(chip);

    chip->release_gpio_chip = free_gpio_chip;

    return chip;

}

GPIO *gpio_force_init(const GPIO_CHIP *chip, const int number) {
    /* some sanity checks */
    if (!chip) {
        return NULL;
    }
    if (0 > number || number > chip->num_gpios - 1) {
        return NULL;
    }

    GPIO *gpio = malloc(sizeof(GPIO));

    /* gpio number? */
    gpio->number = (unsigned)number;

    /* gpio chip? */
    gpio->chip = chip;

    /* gpio bank? */
    if (number < RP1_BANK_BASE[1]) {
        gpio->bank = 0;
    }else if (number < RP1_BANK_BASE[2]) {
        gpio->bank = 1;
    }else
        gpio->bank = 2;

    /* offset? */
    gpio->offset = number - RP1_BANK_BASE[gpio->bank];

    /* architectural name? */
    static char arch_name[16];
    sprintf(arch_name, "GPIO%d", number);
    gpio->arch_name = arch_name;

    /* gpio name in device tree? */
    uint64_t names_len = 2;
    char *names = dt_read_prop(chip->device_tree_node, "gpio-line-names", &names_len);
    char *token = names;
    for (int tally = 0; token != NULL; ++tally) {
        if (tally == number) {
            gpio->name = strdup(token);
            break;
        }
        token = token + strlen(token) + 1; // skip null terminator
    }
    dt_free(names);

    /* add methods. */
    gpio->set_function = set_function;
    gpio->get_function = get_function;
    gpio->set_direction = set_direction;
    gpio->get_direction = get_direction;
    gpio->set_function = set_function;
    gpio->get_function = get_function;
    gpio->set_drive = set_drive;
    gpio->get_level = get_level;
    gpio->get_pull = get_pull;
    gpio->set_pull = set_pull;
    gpio->release_gpio = free_gpio;
    return gpio;
}

GPIO *gpio_init(const GPIO_CHIP *chip, const int number) {
    if (!IS_HEADER_PIN(number)) {
        fprintf(stderr,"GPIO pin failed: %d. Use force_gpio_init for non-header pins.\n", number);
        return NULL;
    }
    return gpio_force_init(chip, number);
}
