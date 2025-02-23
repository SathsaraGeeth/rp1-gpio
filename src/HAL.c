/**
 * @brief Userspace gpio driver for Raspberry Pi devices with RP1 South Bridge chip.
 * @author Geeth Sathsara
 * @date 2024
 */


#include <stdio.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "util.h"
#include "gpio.h"


/* Hardware Based */

int digitalWrite(GPIO *gpio, int value) {
    if (!gpio || !(value == 0 || value == 1)) {
        printf("we are heer");
        return EINVAL;
    } else {
        gpio->set_drive(gpio, (value) ? DRIVE_HIGH : DRIVE_LOW);
        return 0;
    }
}

int digitalRead(GPIO *gpio) {
    if (!gpio/* (gpio && gpio->get_function(gpio) == FN_INPUT) */) {
        return EINVAL;
    }
    return (gpio->get_level(gpio)) ? HIGH : LOW;
}

int pinPull(GPIO *gpio, int direction) {
    if (!gpio || !(direction == UP || direction == DOWN || direction == NONE)) {
        return EINVAL;
    }
    PULL pull;
    if (direction == UP) {
        pull = PULL_UP;
    } else if (direction == DOWN) {
        pull = PULL_DOWN;
    } else {
        pull = PULL_NONE;
    }
    gpio->set_pull(gpio, pull);
    return 0;
}

int readPull(GPIO *gpio) {
    if (!gpio) {
        return EINVAL;
    }
    PULL pull = gpio->get_pull(gpio);
    if (pull == PULL_UP) {
        return 1;
    }
    if (pull == PULL_DOWN) {
        return 0;
    }
    if (pull == PULL_NONE) {
        return -1;
    }
    return EINVAL;
}

int pinMode(GPIO *gpio, int mode) {
    if (!gpio || !(mode == INPUT || mode == OUTPUT || mode == NONE)) {
        return EINVAL;
    }
    if (mode == OUTPUT) {
        FUNCTION func = FN_OUTPUT;
        gpio->set_function(gpio, func);
        return 0;
    } else if (mode == INPUT) {
        FUNCTION func = FN_INPUT;
        gpio->set_function(gpio, func);
        return 0;
    } else {
        FUNCTION func = FN_NONE;
        gpio->set_function(gpio, func);
        return 0;
    }
}

void release_gpio(GPIO *gpio) {
    gpio->release_gpio(gpio);
}

void release_gpio_chip(GPIO_CHIP *chip) {
    chip->release_gpio_chip(chip);
}
