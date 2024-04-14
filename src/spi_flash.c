/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Example of reading/writing an external serial flash using the PL022 SPI interface

#include "spi_flash.h"
#include "boards/pico_w.h"
#include "hardware/spi.h"
#include "pico/time.h"
#include "pico/types.h"

static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void __not_in_flash_func(flash_read)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t* dest, size_t len) {
    cs_select(cs_pin);
    uint8_t cmdbuf[4] = {
            FLASH_CMD_READ,
            addr >> 16,
            addr >> 8,
            addr
    };
    spi_write_blocking(spi, cmdbuf, 4);
    spi_read_blocking(spi, 0, dest, len);
    cs_deselect(cs_pin);
}

void __not_in_flash_func(flash_write_enable)(spi_inst_t *spi, uint cs_pin) {
    cs_select(cs_pin);
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    spi_write_blocking(spi, &cmd, 1);
    cs_deselect(cs_pin);
}

void __not_in_flash_func(flash_wait_done)(spi_inst_t *spi, uint cs_pin) {
    uint8_t status;
    do {
        cs_select(cs_pin);
        uint8_t buf[2] = {FLASH_CMD_STATUS, 0};
        spi_write_read_blocking(spi, buf, buf, 2);
        cs_deselect(cs_pin);
        status = buf[1];
    } while (status & FLASH_STATUS_BUSY_MASK);
}

void __not_in_flash_func(flash_sector_erase)(spi_inst_t *spi, uint cs_pin, uint32_t addr) {
    uint8_t cmdbuf[4] = {
            FLASH_CMD_SECTOR_ERASE,
            addr >> 16,
            addr >> 8,
            addr
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void __not_in_flash_func(flash_block_erase)(spi_inst_t *spi, uint cs_pin, uint32_t addr) {
    uint8_t cmdbuf[4] = {
            FLASH_CMD_BLOCK_ERASE,
            addr >> 16,
            addr >> 8,
            addr
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}


void __not_in_flash_func(flash_erase)(spi_inst_t *spi, uint cs_pin) {
    uint8_t cmdbuf[1] = {
            FLASH_CMD_CHIP_ERASE
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 1);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void __not_in_flash_func(flash_page_program)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t* src) {
    uint8_t cmdbuf[4] = {
            FLASH_CMD_PAGE_PROGRAM,
            addr >> 16,
            addr >> 8,
            addr
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    spi_write_blocking(spi, src, FLASH_PAGE_SIZE);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void __not_in_flash_func(flash_write)(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t* src, size_t size) {
    uint8_t cmdbuf[4] = {
            FLASH_CMD_PAGE_PROGRAM,
            addr >> 16,
            addr >> 8,
            addr
    };
    flash_write_enable(spi, cs_pin);
    cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, 4);
    spi_write_blocking(spi, src, size);
    cs_deselect(cs_pin);
    flash_wait_done(spi, cs_pin);
}

void write_entry(uint8_t data_entry[PACKET_SIZE]) {
    flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, base_addr, page_buffer, FLASH_PAGE_SIZE);
    for (uint16_t i = 0; i < FLASH_PAGE_SIZE; i += PACKET_SIZE) {
        if (page_buffer[i] == 0xFF) {
            base_addr += i;
            break;
        }
        if ((i + PACKET_SIZE) == FLASH_PAGE_SIZE) {
            base_addr += FLASH_PAGE_SIZE;
            flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, base_addr, page_buffer, FLASH_PAGE_SIZE);
            i = 0;
        }
    }
    flash_write(spi_default, PICO_DEFAULT_SPI_CSN_PIN, base_addr, data_entry, PACKET_SIZE);
    base_addr += PACKET_SIZE;
}


#ifdef FLASH_TEST
    #include "pico/multicore.h"
    #include <stdint.h>
    #include <inttypes.h>
    #include <stdio.h>


    void printbuf(uint8_t buf[FLASH_PAGE_SIZE]) {
        for (int i = 0; i < FLASH_PAGE_SIZE; ++i) {
            if (i % 16 == 15)
                printf("%02x\n", buf[i]);
            else
                printf("%02x ", buf[i]);
        }
    }



    void core1_entry() {
        printf("SPI flash example\n");

        // Enable SPI 0 at 1 MHz and connect to GPIOs
        spi_init(spi_default, 1000 * 1000 * 60);
        gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
        gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
        gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);

        // Chip select is active-low, so we'll initialise it to a driven-high state
        gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
        gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
        gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

        // flash_erase(spi_default, PICO_DEFAULT_SPI_CSN_PIN);
        // flash_sector_erase(spi_default, PICO_DEFAULT_SPI_CSN_PIN, 0);
        printf("Before program:\n");
        flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, 0, page_buffer, FLASH_PAGE_SIZE);
        printbuf(page_buffer);

        uint8_t entry[PACKET_SIZE];

        printf("Written Data:\n");
        printf("time (us)\t|\tstate\t|\tdep pcnt\t|\talt (m)\t|\tvel (m/s)\t|\tempty\n");
        for (uint16_t i = 0; i < 500; i++) {
            absolute_time_t now = get_absolute_time();
            uint64_t now_us= to_us_since_boot(now);
            float altitude = 10.0f * i;
            float velocity = 5.0f * i;
            uint8_t deploy_percent = (i*1000) / 200;
            printf("%"PRIu64"\t|\t%"PRIu8"\t|\t%"PRIu8"\t|\t%4.2f\t|\t%4.2f\t|\tDAWSYN_SCHRAIB\n", now_us, (uint8_t)(i), deploy_percent, altitude, velocity);
            uint32_t alt_bits = *((uint32_t *)&altitude);
            uint32_t vel_bits = *((uint32_t *)&velocity);
            entry[0] = now_us >> 56;
            entry[1] = now_us >> 48;
            entry[2] = now_us >> 40;
            entry[3] = now_us >> 32;
            entry[4] = now_us >> 24;
            entry[5] = now_us >> 16;
            entry[6] = now_us >> 8;
            entry[7] = now_us;
            entry[8] = i;
            entry[9] = deploy_percent;
            entry[10] = alt_bits >> 24;
            entry[11] = alt_bits >> 16;
            entry[12] = alt_bits >> 8;
            entry[13] = alt_bits;
            entry[14] = vel_bits >> 24;
            entry[15] = vel_bits >> 16;
            entry[16] = vel_bits >> 8;
            entry[17] = vel_bits;
            entry[18] = 'D';
            entry[19] = 'A';
            entry[20] = 'W';
            entry[21] = 'S';
            entry[22] = 'Y';
            entry[23] = 'N';
            entry[24] = '_';
            entry[25] = 'S';
            entry[26] = 'C';
            entry[27] = 'H';
            entry[28] = 'R';
            entry[29] = 'A';
            entry[30] = 'I';
            entry[31] = 'B';
            write_entry(entry);
        }

        printf("After program:\n");
        flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, 0, page_buffer, FLASH_PAGE_SIZE);
        printbuf(page_buffer);

        printf("\nRead Data:\n");
        printf("time (us)\t|\tstate\t|\tdep pcnt\t|\talt (m)\t|\tvel (m/s)\t|\tempty\n");
        for (uint32_t i = 0; i < base_addr; i += PACKET_SIZE) {
            flash_read(spi_default, PICO_DEFAULT_SPI_CSN_PIN, i, entry, PACKET_SIZE);
            uint64_t now_us = (((uint64_t)entry[0] << 56) | ((uint64_t)entry[1] << 48) | \
                              ((uint64_t)entry[2] << 40)  | ((uint64_t)entry[3] << 32) | \
                              ((uint64_t)entry[4] << 24)  | ((uint64_t)entry[5] << 16) | \
                              ((uint64_t)entry[6] << 8)   | ((uint64_t)entry[7]));

            uint8_t state = entry[8];
            uint8_t deploy_percent = entry[9];

            uint32_t alt_bits = (entry[10] << 24) | (entry[11] << 16) | (entry[12] << 8) | (entry[13]);
            uint32_t vel_bits = (entry[14] << 24) | (entry[15] << 16) | (entry[16] << 8) | (entry[17]);
            float altitude = *(float *)(&alt_bits);
            float velocity = *(float *)(&vel_bits);
            printf("%"PRIu64"\t|\t%"PRIu8"\t|\t%"PRIu8"\t|\t%4.2f\t|\t%4.2f\t|\t%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n", \
                    now_us, state, deploy_percent, altitude, velocity, \
                    entry[18],entry[19],entry[20],entry[21],entry[22],entry[23],entry[24],entry[25],entry[26],entry[27],entry[28],entry[29],entry[30],entry[31]);
        }

    }

    int main() {
        // Enable UART so we can print status output
        stdio_init_all();
        getchar();

        multicore_launch_core1(core1_entry);

        while (1) {
            tight_loop_contents();
        }

    }
#endif
