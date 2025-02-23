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



























int main() {
    GPIO_CHIP *chip = gpio_chip_init();

    UNUSED(chip); // @debug
    // @debug ->
    printf("________________ %s _________________\n", "GPIO CHIP");
    printf("gpio chip name: %s\n", chip->name);
    printf("gpio chip compatible: %s\n", chip->compatible);
    printf("gpio chip size:  0x%x\n", chip->size);
    printf("gpio chip number of gpio's: %d\n", chip->num_gpios);
    printf("gpio chip device tree node: %s\n", chip->device_tree_node);
    printf("gpio chip memory fd: %d\n", chip->mem_fd);
    // printf("gpio chip physical address: %" PRIu64 "\n", chip->physical_address);
    printf("gpio chip physical address: 0x%" PRIu64 "\n", chip->physical_address);
    // printf("gpio chip virtual address: %p\n", chip->virtual_address);
    // printf("gpio chip base address: %" PRIu32 "\n", chip->base_address);
    printf("gpio chip base address(ptr): %p\n", (void *)chip->base_address);
    printf("gpio chip base address: 0x%lx\n", (unsigned long)(*(chip->base_address)));


    // <- @debug

    GPIO *gpioX = gpio_init(chip, 12);
    // @debug ->
    printf("________________ %s _________________\n", "GPIO PIN");
    printf("gpio name: %s\n", gpioX->name);
    printf("gpio arch name: %s\n", gpioX->arch_name);
    printf("gpio number:  %d\n", gpioX->number);
    printf("gpio chip (ptr): %p\n", (void *)(gpioX->chip));
    printf("bank: %d\n", gpioX->bank);
    printf("offset: %d\n", gpioX->offset);
    // <- @debug

    // gpioX->set_function(gpioX, FN_OUTPUT);
    // gpioX->set_direction(gpioX, DIR_OUTPUT);
    pinMode(gpioX, OUTPUT);
    // gpioX->set_drive(gpioX, 1);
    for (int i = 0; i <= 3; i++) {
        digitalWrite(gpioX, HIGH);
        // start_pwm(gpioX, 50, 25);
        usleep(250000);
        // stop_pwm(gpioX);
        // gpioX->set_drive(gpioX, 0);
        // sleep(1);
        digitalWrite(gpioX, LOW);
        usleep(250000);
    }
    // software_pwm(gpioX, 0.5, 0);



    release_gpio(gpioX);
    release_gpio_chip(chip);

    return 0;
}
