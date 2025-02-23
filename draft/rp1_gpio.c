#include "rp1_gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <errno.h>


uint32_t *memory_mapped_address(uint32_t physical_address, size_t size, size_t *map_size);
void unmap_memory(void *mapped_base, size_t map_size);

uint32_t *memory_mapped_address(uint32_t physical_address, size_t size, size_t *map_size) {
    int page_size = getpagesize();
    int fd;
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Failed to open /dev/mem");
        return NULL;
    }
    uint64_t aligned_address = physical_address & ~(page_size - 1); // aligned to nearest page boundary
    uint64_t offset = physical_address - aligned_address; // offset due to alignment
    *map_size = size + offset;
    void *map = mmap(
       NULL,                    // OS choose the virtual address
       *map_size,
       PROT_READ | PROT_WRITE,  // RW permissions
       MAP_SHARED,              // Shared with other processes
       fd,                      // File descriptor for /dev/mem
       aligned_address          // Aligned physical address
    );
    close(fd);
    if (map == MAP_FAILED) {
        perror("Failed to mmap");
        return NULL;
    }
    return (uint32_t *)((char *)map + offset);
}

void unmap_memory(void *mapped_base, size_t map_size) {
    if (mapped_base) {
        munmap(mapped_base, map_size);
    }
}

// int main() {
//     int gpio_num = 12;
//     size_t mapped_size;
//     uint32_t base = memory_mapped_address(CHIP_BASE_ADDRESS, CHIP_SIZE, &mapped_size);
//
//     SET_FN_GPIO(base, gpio_num);
//
//
//     return 0;
// }

int main() {
    size_t mapped_size;
    printf("Physical address: 0x%08x\n", CHIP_BASE_ADDRESS);
    uint32_t *base = memory_mapped_address(CHIP_BASE_ADDRESS, CHIP_SIZE, &mapped_size);
    // uint32_t base = CHIP_BASE_ADDRESS;
    printf("Mapped base address: 0x%08x\n", /* */*base);
    int gpio_num = 4;
    int bank_num = GPIOn_BANK(gpio_num);
    printf("GPIO_num: %d\n", gpio_num);
    printf("GPIOn_BANK: %u\n", bank_num);
    //
    volatile u_int32_t io_bank_base_addr = IO_BANK_BASE_ADDR(base, bank_num);
    printf("IO_BANK_BASE_ADDR: 0x%08x\n", io_bank_base_addr);
    volatile u_int32_t sys_rio_base_addr = SYS_RIO_BANK_BASE_ADDR(base, bank_num);
    printf("SYS_RIO_BANK_BASE_ADDR: 0x%08x\n", sys_rio_base_addr);
    volatile u_int32_t pads_bank_base_addr = PADS_BANK_BASE_ADDR(base, bank_num);
    printf("PADS_BANK_BASE_ADDR: 0x%08x\n", pads_bank_base_addr);

    printf("is valid gpio: %d\n", IS_VALID_GPIO(gpio_num));
    printf("is header gpio: %d\n", IS_HEADER_GPIO(gpio_num));


    // SET_FN_GPIO(base, gpio_num);
//     volatile u_int32_t b = 0x07140007;
//     REG_CLEAR_BITS(b + GPIOn_CTRL_OFFSET(gpio_num), FN_CLR_MASK);
//     REG_SET_BITS(b + GPIOn_CTRL_OFFSET(gpio_num), SYS_RIO_FN_MASK);
//     // GPIO_SET_OUTPUT(gpio_num);
//     *REG_ADDR(base, RIO_OE_OFFSET + CLR_OFFSET) = GPIOn_INTERFACE_MASK(gpio_num);
}
