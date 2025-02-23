# GPIO Library

A simple C library for Raspberry Pi devices with RP1 south bridge chip (currently Raspberry Pi 5) GPIO control with a Hardware Abstraction Layer (HAL).

## Build
```sh
make
```
## Complile with the Library
```sh
gcc -Iinclude examples/main.c -L. -lgpio -o example
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
./example
```
