/* Blinky! - blink an led conneted to gpio 12 10 times. */

#include "HAL.h"
#include <unistd.h>


int main() {
    // first need to initialize the gpio chip
    GPIO_CHIP *chip = gpio_chip_init(); // gpio_chip_init() returns a pointer to the gpio chip

    // next we can initialize a gpio pin (12 in this example) in the gpio chip
    GPIO *gpioX = gpio_init(chip, 12); // gpio_init() returns a pointer to the gpio pin
    
    // set gpio mode to OUTPUT
    pinMode(gpioX, OUTPUT);
  
    
    for (int i = 0; i <= 10; i++) {
        // set the gpio pin 12 logical HIGH
        digitalWrite(gpioX, HIGH);
        usleep(250000);
        // set the gpio pin 12 logical LOW
        digitalWrite(gpioX, LOW);
        usleep(250000);
    }
  
    // release the resources associated with gpio chip and gpio pin
    release_gpio(gpioX);
    release_gpio_chip(chip);

    return 0;
}
