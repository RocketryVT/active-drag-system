/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <inttypes.h>
#include "boards/pico_w.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"

#define ADDR 0x60
#define MAX_SCL 400000
#define DATA_RATE_HZ 100

bool timer_callback(repeating_timer_t *rt);

// volatile float velocity = 0.0f;
volatile float altitude = 0;

int main() {
    stdio_init_all();

    repeating_timer_t timer;
    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    uint8_t config[2] = {0};
	// Select control register(0x26)
	// Active mode, OSR = 128, altimeter mode(0xB9)
    config[0] = 0x26;
    config[1] = 0xB9;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);
	// Select data configuration register(0x13)
	// Data ready event enabled for altitude, pressure, temperature(0x07)
    config[0] = 0x13;
    config[1] = 0x07;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    uint8_t reg[1] = {0};
    i2c_write_blocking(i2c_default, ADDR, reg, 1, false);
    uint8_t data[4] = {0};
    i2c_read_blocking(i2c_default, ADDR, data, 4, false);
    // Initial Reading
    uint32_t temp_alt = (data[1] << 24) | (data[2] << 16) | (data[3] << 8);
    altitude = temp_alt / 65536.0f;

    if (!add_repeating_timer_us(-1000000 / DATA_RATE_HZ,  timer_callback, NULL, &timer)) {
        printf("Failed to add timer!\n");
        return 1;
    }
    while (1) {
        tight_loop_contents();
    }
}

bool timer_callback(repeating_timer_t *rt) {
    static float prev_altitude = altitude;
    absolute_time_t last = get_absolute_time();
    uint8_t reg[1] = {0};
    i2c_write_blocking(i2c_default, ADDR, reg, 1, false);
    uint8_t data[4] = {0};
    i2c_read_blocking(i2c_default, ADDR, data, 4, false);
    // Exactly how MPL3115A2 datasheet says to retrieve altitude
    uint32_t temp_alt = (data[1] << 24) | (data[2] << 16) | (data[3] << 8);
    altitude = temp_alt / 65536.0f;
    printf("Altitude: %4.2f\n", altitude);
    printf("Velocity: %4.2f\n", ((altitude - prev_altitude) / 0.01f));
    prev_altitude = altitude;
    absolute_time_t now = get_absolute_time();
    int64_t time_delta = absolute_time_diff_us(last, now);
    printf("Time Delta: %" PRIi64"\n", time_delta);
    return true;
}
