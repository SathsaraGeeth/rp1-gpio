#ifndef RP1_GPIO_H
#define RP1_GPIO_H


#include <stdbool.h>





/* ------------------------------------------------------------------------------------------------------------------ */

#define REG_ADDR(base, offset)                 ((volatile uint32_t *)((base) + (offset)))

#define REG_WRITE(reg, value)                  (*(reg) = (value))
#define REG_READ(reg)                          (*(reg))
#define REG_SET_BITS(reg, mask)                (*(reg) |= (mask))
#define REG_CLEAR_BITS(reg, mask)              (*(reg) &= ~(mask))

// #define REG_WRITE(reg, value)                  (reg = (value))
// #define REG_READ(reg)                          (reg)
// #define REG_SET_BITS(reg, mask)                (reg |= (mask))
// #define REG_CLEAR_BITS(reg, mask)              (reg &= ~(mask))

// #define _write32(base, peri_offset, reg_offset, value) \
// base[(peri_offset + reg_offset)/4] = value
//
// #define _read32(base, peri_offset, reg_offset) \
// base[(peri_offset + reg_offset)/4]


/* ------------------------------------------------------------------------------------------------------------------ */

#define CHIP_NAME                              "rp1"
#define CHIP_COMPATIBLE                        "raspberrypi,rp1-gpio"
#define CHIP_SIZE                              0x000030000
#define NUM_GPIO                               54
#define NUM_BANKS                              3

static const int BANK_BASE[NUM_BANKS]    =     {0, 28, 34};

#define GPIOn_BANK(n) \
((n) < BANK_BASE[1] ? 0 : ((n) < BANK_BASE[2] ? 1 : 2))

#define NUM_HEADER_GPIO                        28

#define IS_VALID_GPIO(n) \
!!((n >= 0) && (n < NUM_GPIO))

#define IS_HEADER_GPIO(n) \
!!(1 < n && (n < NUM_HEADER_GPIO))


#define IO_BANK0_BASE_ADDR                     0x400d0000
#define IO_BANK1_BASE_ADDR                     0x400d4000
#define IO_BANK2_BASE_ADDR                     0x400d8000
#define SYS_RIO_BANK0_BASE_ADDR                0x400e0000
#define SYS_RIO_BANK1_BASE_ADDR                0x400e4000
#define SYS_RIO_BANK2_BASE_ADDR                0x400e8000
#define PADS_BANK0_BASE_ADDR                   0x400f0000
#define PADS_BANK1_BASE_ADDR                   0x400f4000
#define PADS_BANK2_BASE_ADDR                   0x400f8000



#define CHIP_BASE_ADDRESS                      IO_BANK0_BASE_ADDR



#define IO_BANK0_OFFSET                        IO_BANK0_BASE_ADDR          -        IO_BANK0_BASE_ADDR
#define IO_BANK1_OFFSET                        IO_BANK1_BASE_ADDR          -        IO_BANK0_BASE_ADDR
#define IO_BANK2_OFFSET                        IO_BANK2_BASE_ADDR          -        IO_BANK0_BASE_ADDR
#define SYS_RIO_BANK0_OFFSET                   SYS_RIO_BANK0_BASE_ADDR     -        IO_BANK0_BASE_ADDR
#define SYS_RIO_BANK1_OFFSET                   SYS_RIO_BANK1_BASE_ADDR     -        IO_BANK0_BASE_ADDR
#define SYS_RIO_BANK2_OFFSET                   SYS_RIO_BANK2_BASE_ADDR     -        IO_BANK0_BASE_ADDR
#define PADS_BANK0_OFFSET                      PADS_BANK0_BASE_ADDR        -        IO_BANK0_BASE_ADDR
#define PADS_BANK1_OFFSET                      PADS_BANK1_BASE_ADDR        -        IO_BANK0_BASE_ADDR
#define PADS_BANK2_OFFSET                      PADS_BANK2_BASE_ADDR        -        IO_BANK0_BASE_ADDR


/* ------------------------------------------------------------------------------------------------------------------ */

// #define IO_BANK_BASE_ADDR(mapped_base_addr, bank) \
// REG_ADDR(mapped_base_addr, IO_BANK##bank##_OFFSET)
#define IO_BANK_BASE_ADDR(mapped_base_addr, bank) \
( \
(bank) == 0 ? REG_ADDR(mapped_base_addr, IO_BANK0_OFFSET) : \
(bank) == 1 ? REG_ADDR(mapped_base_addr, IO_BANK1_OFFSET) : \
(bank) == 2 ? REG_ADDR(mapped_base_addr, IO_BANK2_OFFSET) : \
NULL /* Return NULL or handle invalid bank case */ \
)

// to be implemented interrupts and advanced functions
#define SYS_RIO_FN_MASK                        (1 << 5)
#define FN_CLR_MASK                            0x11111

#define GPIOn_CTRL_OFFSET(n)                   (0x04 * (2 * n + 1))
#define GPIOn_Status_OFFSET(n)                 (0x04 * 2 * n)


// set gpio mode
#define SET_FN_GPIO(mapped_base_addr, gpio_num) \
REG_CLEAR_BITS(IO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_CTRL_OFFSET(gpio_num), FN_CLR_MASK); \
REG_SET_BITS(IO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num) + GPIOn_CTRL_OFFSET(gpio_num), SYS_RIO_FN_MASK)
\


/* ------------------------------------------------------------------------------------------------------------------ */

/* GPIO Operations:

 * Set GPIO HIGH:
 * Write (1 << gpio_num) to
 * SYS_RIO0_BASE_ADDR + RIO_OUT_OFFSET + SET_OFFSET
 *
 * Set GPIO LOW:
 * Write (1 << gpio_num) to
 * SYS_RIO0_BASE_ADDR + RIO_OUT_OFFSET + CLR_OFFSET
 *
 * Set GPIO as INPUT:
 * Write (1 << gpio_num) to
 * SYS_RIO0_BASE_ADDR + RIO_OE_OFFSET + SET_OFFSET
 *
 * Set GPIO as OUTPUT:
 * Write (1 << gpio_num) to
 * SYS_RIO0_BASE_ADDR + RIO_OE_OFFSET + CLR_OFFSET
 *
 * Read GPIO mode (INPUT/OUTPUT):
 * Read SYS_RIO0_BASE_ADDR + RIO_OE_OFFSET,
 * mask with (1 << gpio_num), and invert the result.
 *
 * Read GPIO logical value (HIGH/LOW):
 * Read SYS_RIO0_BASE_ADDR + RIO_SYNC_IN_OFFSET,
 * mask with (1 << gpio_num), and invert the result.
 */


// #define SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, bank) \
// REG_ADDR(mapped_base_addr, SYS_RIO_BANK##bank##_OFFSET)

#define SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, bank) \
( \
(bank) == 0 ? REG_ADDR(mapped_base_addr, SYS_RIO_BANK0_OFFSET) : \
(bank) == 1 ? REG_ADDR(mapped_base_addr, SYS_RIO_BANK1_OFFSET) : \
(bank) == 2 ? REG_ADDR(mapped_base_addr, SYS_RIO_BANK2_OFFSET) : \
NULL /* Return NULL or handle invalid bank case */ \
)

#define RIO_OUT_OFFSET                         0x0
#define RIO_OE_OFFSET                          0x4
#define RIO_SYNC_IN_OFFSET                     0x8
#define RIO_NOSYNC_IN_OFFSET                   0xc                                                                      // couldn't find anywhere so I guessed

#define RP1_RW_OFFSET			               0x0000                                                                   // don't know how to use
#define RP1_XOR_OFFSET			               0x1000                                                                   // don't know how to use
#define SET_OFFSET			                   0x2000
#define CLR_OFFSET			                   0x3000


#define GPIOn_INTERFACE_MASK(n)                (1U << (n - GPIOn_BANK(n)))

#define SET_DIGITAL_HIGH_OFFSET                RIO_OUT_OFFSET         +          SET_OFFSET
#define SET_DIGITAL_LOW_OFFSET                 RIO_OUT_OFFSET         +          CLR_OFFSET
#define SET_MODE_INPUT_OFFSET                  RIO_OE_OFFSET          +          SET_OFFSET
#define SET_MODE_OUTPUT_OFFSET                 RIO_OE_OFFSET          +          CLR_OFFSET
#define READ_MODE_OFFSET                       RIO_OE_OFFSET
#define READ_LOGICAL_LEVEL_OFFSET              RIO_SYNC_IN_OFFSET                                                       // NOSYNC will do too but little different how sampling occur; see datasheet


// Set GPIO HIGH
#define GPIO_SET_HIGH(mapped_base_addr, gpio_num) \
(*REG_ADDR(SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)), RIO_OUT_OFFSET + SET_OFFSET) = GPIOn_INTERFACE_MASK(gpio_num))

// Set GPIO LOW
#define GPIO_SET_LOW(mapped_base_addr, gpio_num) \
(*REG_ADDR(SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)), RIO_OUT_OFFSET + CLR_OFFSET) = GPIOn_INTERFACE_MASK(gpio_num))

// Set GPIO as INPUT
#define GPIO_SET_INPUT(mapped_base_addr, gpio_num) \
(*REG_ADDR(SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)), RIO_OE_OFFSET + SET_OFFSET) = GPIOn_INTERFACE_MASK(gpio_num))

// Set GPIO as OUTPUT
#define GPIO_SET_OUTPUT(mapped_base_addr, gpio_num) \
(*REG_ADDR(SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)), RIO_OE_OFFSET + CLR_OFFSET) = GPIOn_INTERFACE_MASK(gpio_num))

// Read GPIO mode (INPUT/OUTPUT)
// Returns 1 if INPUT, 0 if OUTPUT
#define GPIO_READ_MODE(mapped_base_addr, gpio_num) \
(!( (*REG_ADDR(SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)), RIO_OE_OFFSET)) & GPIOn_INTERFACE_MASK(gpio_num)))

// Read GPIO logical value (HIGH/LOW)
// Returns 1 if HIGH, 0 if LOW
#define GPIO_READ_VALUE(mapped_base_addr, gpio_num) \
(!!( (*REG_ADDR(SYS_RIO_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)), RIO_SYNC_IN_OFFSET)) & GPIOn_INTERFACE_MASK(gpio_num)))



/* ------------------------------------------------------------------------------------------------------------------ */
/* GPIO set up: drive mode/slew rate/Schmitt trigger/drive intensity. */

// #define PADS_BANK_BASE_ADDR(mapped_base_addr, bank) \
// REG_ADDR(mapped_base_addr, PADS_BANK##bank##_OFFSET)

#define PADS_BANK_BASE_ADDR(mapped_base_addr, bank) \
( \
(bank) == 0 ? REG_ADDR(mapped_base_addr, PADS_BANK0_OFFSET) : \
(bank) == 1 ? REG_ADDR(mapped_base_addr, PADS_BANK1_OFFSET) : \
(bank) == 2 ? REG_ADDR(mapped_base_addr, PADS_BANK2_OFFSET) : \
NULL /* Return NULL or handle invalid bank case */ \
)


#define VOLTAGE_SELECT_OFFSET                  0x00

#define PAD_VOLTAGE_3V3_MASK                   (0)                                                                      // or the voltage clr mask as 3.3v is the default
#define PAD_VOLTAGE_1V8_MASK                   (1 << 0)


#define GPIOn_PAD_CTR_OFFSET(n)                (0x04 + (0x04 * (n)))

#define PAD_OUTPUT_DISABLE_MASK                (1 << 7)
#define PAD_INPUT_ENABLE_MASK                  (1 << 6)
#define DRIVE_2MA_MASK                         (0)
#define DRIVE_4MA_MASK                         (1 << 4)
#define DRIVE_8MA_MASK                         (1 << 5)
#define DRIVE_12MA_MASK                        ((1 << 4) | (1 << 5))
#define PAD_PULL_UP_ENABLE_MASK                (1 << 3)
#define PAD_PULL_DOWN_ENABLE_MASK              (1 << 2)
#define PAD_SCHMITT_ENABLE_MASK                (1 << 1)
#define SLEW_FAST_MASK                         (1 << 0)
#define SLEW_SLOW_MASK                         (0)


// INPUT/OUTPUT
#define ENABLE_OUTPUT(mapped_base_addr, gpio_num) \
REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_OUTPUT_DISABLE_MASK)

#define DISABLE_OUTPUT(mapped_base_addr, gpio_num) \
REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_OUTPUT_DISABLE_MASK)

#define ENABLE_INPUT(mapped_base_addr, gpio_num) \
REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_INPUT_ENABLE_MASK)

#define DISABLE_INPUT(mapped_base_addr, gpio_num) \
REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_INPUT_ENABLE_MASK)

// Enable/Disable Pull-Up
#define ENABLE_PULL_UP(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_PULL_UP_ENABLE_MASK)

#define DISABLE_PULL_UP(mapped_base_addr, gpio_num) \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_PULL_UP_ENABLE_MASK)

// Enable/Disable Pull-Down
#define ENABLE_PULL_DOWN(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_PULL_DOWN_ENABLE_MASK)

#define DISABLE_PULL_DOWN(mapped_base_addr, gpio_num) \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_PULL_DOWN_ENABLE_MASK)

// Enable/Disable Schmitt Trigger
#define ENABLE_SCHMITT(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_SCHMITT_ENABLE_MASK)

#define DISABLE_SCHMITT(mapped_base_addr, gpio_num) \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), PAD_SCHMITT_ENABLE_MASK)

// Set Drive Strength
#define SET_DRIVE_2MA(mapped_base_addr, gpio_num) \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), (DRIVE_4MA_MASK | DRIVE_8MA_MASK))

#define SET_DRIVE_4MA(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), DRIVE_4MA_MASK); \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), DRIVE_8MA_MASK)

#define SET_DRIVE_8MA(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), DRIVE_8MA_MASK); \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), DRIVE_4MA_MASK)

#define SET_DRIVE_12MA(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), (DRIVE_4MA_MASK | DRIVE_8MA_MASK))

// Set Slew Rate
#define SET_SLEW_FAST(mapped_base_addr, gpio_num) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), SLEW_FAST_MASK)

#define SET_SLEW_SLOW(mapped_base_addr, gpio_num) \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + GPIOn_PAD_CTR_OFFSET(gpio_num), SLEW_FAST_MASK)

// Set Voltage Level
#define SET_PAD_VOLTAGE_3V3(mapped_base_addr) \
    REG_CLEAR_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + VOLTAGE_SELECT_OFFSET, PAD_VOLTAGE_1V8_MASK)

#define SET_PAD_VOLTAGE_1V8(mapped_base_addr) \
    REG_SET_BITS(PADS_BANK_BASE_ADDR(mapped_base_addr, GPIOn_BANK(gpio_num)) + VOLTAGE_SELECT_OFFSET, PAD_VOLTAGE_1V8_MASK)

/* ------------------------------------------------------------------------------------------------------------------ */




/* ------------------------------------------------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------------------------------------------------ */

// typedef struct CHIP {
//     const char *name;
//     const char *base;
//     const int num_gpio_pins;
//     const char *gpio_pins[];
// } CHIP;
//
// typedef struct GPIO_PIN {
//     const char *name;
//     const int gpio_num;
//
//     /* --------- basic gpio interface --------- */
//     void (*)
//
// } GPIO_PIN;





#endif //RP1_GPIO_H
