#ifndef SPI_FLASH
#define SPI_FLASH

#include <stdint.h>
#include "hardware/spi.h"
#include "boards/pico_w.h"
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_PAGE_SIZE        256
#define FLASH_NUM_PAGES        32768
#define FLASH_SECTOR_SIZE      4096
#define FLASH_BLOCK_SIZE       65536
#define FLASH_PHYS_SIZE        (FLASH_PAGE_SIZE * FLASH_NUM_PAGES)

#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_READ         0x03
#define FLASH_CMD_STATUS       0x05
#define FLASH_CMD_WRITE_EN     0x06
#define FLASH_CMD_SECTOR_ERASE 0x20
#define FLASH_CMD_BLOCK_ERASE  0xD8
#define FLASH_CMD_CHIP_ERASE   0xC7

#define FLASH_STATUS_BUSY_MASK 0x01

// #define FLASH_TEST

#define PACKET_SIZE            32


static uint8_t page_buffer[FLASH_PAGE_SIZE];
static uint32_t base_addr = 0;

void write_entry(uint8_t* data_entry);

void __not_in_flash_func(flash_read)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t *dest, size_t len);


void __not_in_flash_func(flash_write_enable)(spi_inst_t *spi, uint cs_pin);
void __not_in_flash_func(flash_page_program)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t* src);
void __not_in_flash_func(flash_write)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t* src, size_t size);

void __not_in_flash_func(flash_wait_done)(spi_inst_t *spi, uint cs_pin);

void __not_in_flash_func(flash_sector_erase)(spi_inst_t *spi, uint cs_pin, uint32_t addr);
void __not_in_flash_func(flash_block_erase)(spi_inst_t *spi, uint cs_pin, uint32_t addr);
void __not_in_flash_func(flash_erase)(spi_inst_t *spi, uint cs_pin);

#ifdef __cplusplus
}
#endif
#endif
