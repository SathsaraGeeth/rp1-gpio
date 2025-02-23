#ifndef GPIO_H
#define GPIO_H

#include <inttypes.h>
#include <unistd.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

#include "HAL.h"


/* Interfacing types */
typedef enum {
    FN0,
    FN1,
    FN2,
    FN3,
    FN4,
    FN5,
    FN6,
    FN7,
    FN8,
    FN_INPUT = 0x10,
    FN_OUTPUT,
    FN_GPIO,           /* Preserves direction if possible, else input */
    FN_NONE,           /* If possible, else input */
    FN_MAX
} FUNCTION;

typedef enum {
    DIR_INPUT,
    DIR_OUTPUT,
    DIR_MAX,
} DIRECTION;

typedef enum {
    DRIVE_LOW,
    DRIVE_HIGH,
    DRIVE_MAX
} DRIVE;

typedef enum {
    PULL_NONE,
    PULL_DOWN,
    PULL_UP,
    PULL_MAX
} PULL;


/**
 * @brief Represents a GPIO chip and its associated resources.
 *
 * This structure defines the properties and operations for a GPIO chip.
 * It includes metadata about the chip, memory mapping details, and methods
 * to manage the chip's lifecycle.
 */
typedef struct GPIO_CHIP {
    const char *name;               /**< Name of the GPIO chip (e.g., "gpiochip0"). */
    const char *compatible;         /**< Compatible string as defined in the device tree. */
    int size;                       /**< Size of memory mapped for the GPIO chip. */
    int num_gpios;                  /**< Total number of GPIO pins available on this chip. */
    const char *device_tree_node;   /**< Device tree node name for the GPIO chip. */
    int mem_fd;                     /**< File descriptor for the memory-mapped GPIO interface. */
    uint64_t physical_address;      /**< Physical address for the GPIO chip's memory mapping. */
    volatile uint32_t *base_address;/**< Base address for accessing GPIO registers. */
    void (*release_gpio_chip)(GPIO_CHIP *chip); /**< Function to release resources associated with the chip. */
} GPIO_CHIP;

/**
 * @brief Represents a GPIO pin and its associated properties and operations.
 *
 * This structure defines the attributes and methods for interacting with an
 * individual GPIO pin. It includes the pin's configuration, its associated
 * GPIO chip, and functions for controlling its state and direction.
 */
typedef struct GPIO {
    const char *name;               /**< Name of the GPIO pin as defined in the device tree. */
    char *arch_name;                /**< Architecture-specific name of the GPIO (e.g., "GPIO5"). */
    unsigned number;                /**< Pin number (typically 0-53 for Raspberry Pi). */
    const GPIO_CHIP *chip;          /**< Pointer to the GPIO chip to which this pin belongs. */
    int bank;                       /**< GPIO bank number. */
    int offset;                     /**< Offset within the bank for this GPIO pin. */

    void (*set_function)(GPIO *gpio, const FUNCTION function); /**< Function to set the pin's mode (e.g., input, output). */
    FUNCTION (*get_function)(GPIO *gpio); /**< Function to get the pin's current mode. */

    void (*set_direction)(GPIO *gpio, DIRECTION direction); /**< Function to set the pin's direction (input or output). */
    DIRECTION (*get_direction)(GPIO *gpio); /**< Function to get the pin's current direction. */

    void (*set_drive)(GPIO *gpio, DRIVE drive); /**< Function to set the pin's drive strength. */
    int (*get_level)(GPIO *gpio); /**< Function to get the pin's current logic level (high or low). */

    PULL (*get_pull)(GPIO *gpio); /**< Function to get the pin's pull state (pull-up, pull-down, or none). */
    void (*set_pull)(GPIO *gpio, PULL pull); /**< Function to set the pin's pull state. */

    void (*release_gpio)(GPIO *gpio); /**< Function to release resources associated with the GPIO pin. */
} GPIO;

/**
 * @brief Initialize an instance of the GPIO chip data type.
 * @return A pointer to the initialized instance of GPIO chip, or NULL if an error occurs.
 */
GPIO_CHIP *gpio_chip_init(void);

/**
 * @brief Initialize an instance of the GPIO data type.
 * This function can only initialize GPIO pins in the RPI 40-pin header,
 * excluding GPIO0 and GPIO1. If you need to initialize a GPIO pin that is
 * not in the 40-pin header or GPIO0/GPIO1, use `gpio_force_init`.
 * @param chip Pointer to the GPIO chip instance to which the pin belongs.
 * @param number The pin number to initialize (e.g., 1 to 40 for the default GPIO header).
 * @return A pointer to the initialized GPIO pin, or NULL on error.
 */
GPIO *gpio_init(const GPIO_CHIP *chip, const int number);


#endif //GPIO_H
