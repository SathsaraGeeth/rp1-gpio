#include "gpio.h"
#include "HAL.h"

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
