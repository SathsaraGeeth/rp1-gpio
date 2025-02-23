//
// Authored by SathsaraGeeth on 1/13/25.
//

#ifndef HAL_H_
#define HAL_H_


#define HIGH 1
#define LOW 0

#define OUTPUT 1
#define INPUT 0

#define UP 1
#define DOWN 0
#define NONE -1

/* --------------------------------------------- Associated Data Types  --------------------------------------------- */

/**
 * @brief Represents a GPIO chip and its associated resources.
 *
 * This structure defines the properties and operations for a GPIO chip.
 * It includes metadata about the chip, memory mapping details, and methods
 * to manage the chip's lifecycle.
 */
typedef struct GPIO_CHIP GPIO_CHIP;

/**
 * @brief Represents a GPIO pin and its associated properties and operations.
 *
 * This structure defines the attributes and methods for interacting with an
 * individual GPIO pin. It includes the pin's configuration, its associated
 * GPIO chip, and functions for controlling its state and direction.
 */
typedef struct GPIO GPIO;

/* ---------------------------------------- Hardware Defined Functionalities ---------------------------------------- */

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

/**
 * @brief Initialize an instance of the GPIO data type.
 * **Warning: Not recommended for general use.**
 * This function allows initialization of GPIO pins outside the default 40-pin header range.
 * @param chip Pointer to the GPIO chip instance to which the pin belongs.
 * @param number The pin number to initialize (this can be a pin outside the default header range).
 * @return A pointer to the initialized GPIO pin, or NULL on error.
 */
GPIO *gpio_force_init(const GPIO_CHIP *chip, const int number);

/**
* @brief Set the GPIO pin to logical HIGH(1) or LOW(0).
* Must set the GPIO pin to OUTPUT mode.
* @param gpio Pointer to the GPIO structure representing the pin.
* @param value The logical level to set: 1 for HIGH, 0 for LOW.
* @return 0 on success, or EINVAL on error.
*/
int digitalWrite(GPIO *gpio, int value);

/**
 * @brief Read the logical level (HIGH or LOW) of the GPIO pin.
 * The GPIO pin must be set to INPUT mode.
 * @param gpio Pointer to the GPIO structure representing the pin.
 * @return The logical level of the GPIO pin (0 -> LOW, 1 -> HIGH), or EINVAL on error.
 */
int digitalRead(GPIO *gpio);

/**
 * @brief Set the pull state (UP, DOWN, or NONE) of the GPIO pin.
 * @param gpio Pointer to the GPIO structure representing the pin.
 * @param direction The pull direction to set:
 *              -> -1 for pull-NONE (disable pull-up and pull-down).
 *              -> 1 for pull-up (connects the pin to a HIGH logic level internally).
 *              -> 0 for pull-down (connects the pin to a LOW logic level internally).
 * @return 0 on success, or EINVAL on error.
 */
int pinPull(GPIO *gpio, int direction);

/**
 * @brief Read the current pull state (UP, DOWN, or NONE) of the GPIO pin.
 * @param gpio Pointer to the GPIO structure representing the pin.
 * @return The current pull state of the pin:
 *              -> -1 for pull-NONE (disabled pull-up and pull-down).
 *              -> 1 for pull-UP (connected to a HIGH logic level internally).
 *              -> 0 for pull-DOWN (connected to a LOW logic level internally).
 *         Returns EINVAL on error.
 */
int readPull(GPIO *gpio);

/**
 * @brief Set the function of the GPIO pin (INPUT or OUTPUT).
 * @param gpio Pointer to the GPIO structure representing the pin.
 * @param mode The mode to set:
 *             -> 0 for INPUT.
 *             -> 1 for OUTPUT.
 *             -> -1 for NONE if possible.
 * @return 0 on success, or EINVAL on error.
 */
int pinMode(GPIO *gpio, int mode);

/**
 * @brief Release the memory and reset the GPIO pin.
 * @param gpio Pointer to the GPIO instance to release.
 * @return None.
 */
void release_gpio(GPIO *gpio);

/**
 * @brief Release the memory and reset the GPIO chip.
 * @param chip Pointer to the GPIO chip instance to release.
 * @return None.
 */
void release_gpio_chip(GPIO_CHIP *chip);


/* ---------------------------------------- Software Defined Functionalities ---------------------------------------- */

#endif //HAL_H_
