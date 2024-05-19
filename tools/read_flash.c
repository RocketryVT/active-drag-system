#include <stdio.h>
#include <inttypes.h>
#include "boards/pico_w.h"
#include "hardware/spi.h"
#include "spi_flash.h"

int main() {
    stdio_init_all();
    getchar();
    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 1000 * 60);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    uint8_t entry[PACKET_SIZE];

    // flash_erase(spi_default, PICO_DEFAULT_SPI_CSN_PIN);
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

    printf("\nRead Data:\n");
    printf("time (us) | state | dep pcnt | alt (m) | vel (m/s) | quat_w | quat_x | quat_y | quat_z | lin_ax | lin_ay | lin_az\n");
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

        int16_t w = ((int16_t)entry[18]) | (((int16_t)entry[19]) << 8);
        int16_t x = ((int16_t)entry[20]) | (((int16_t)entry[21]) << 8);
        int16_t y = ((int16_t)entry[22]) | (((int16_t)entry[23]) << 8);
        int16_t z = ((int16_t)entry[24]) | (((int16_t)entry[25]) << 8);
        float qw = ((float)w) / 16384.0; // 2^14 LSB
        float qx = ((float)x) / 16384.0;
        float qy = ((float)y) / 16384.0;
        float qz = ((float)z) / 16384.0;
        int16_t ax = ((int16_t)entry[26]) | (((int16_t)entry[27]) << 8);
        int16_t ay = ((int16_t)entry[28]) | (((int16_t)entry[29]) << 8);
        int16_t az = ((int16_t)entry[30]) | (((int16_t)entry[31]) << 8);
        float lax = ((float)x) / 100.0;
        float lay = ((float)y) / 100.0;
        float laz = ((float)z) / 100.0;
        printf("%"PRIu64" | %c | %"PRIu8" | %4.2f | %4.2f | %4.2f | %4.2f| %4.2f | %4.2f | %4.2f | %4.2f |%4.2f\n", \
                now_us, state, deploy_percent, altitude, velocity, qw, qx, qy, qz, lax, lay, laz);
    }
}
