/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <inttypes.h>
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include <math.h>

#define ADDR 0x60
#define MAX_SCL 400000
#define DATA_RATE_HZ 100
#define INT1_PIN 6 // INT1 PIN on MPL3115A2 connected to GPIO PIN 9 (GP6)

#define MOTOR_BURN_TIME 6200 // Burn time in milliseconds for M1939
typedef enum {
    PAD,
    BOOST,
    COAST,
    APOGEE,
    RECOVERY,
    END
} state_t;

void pad_callback(uint gpio, uint32_t event_mask);
int64_t boost_callback(alarm_id_t id, void* user_data);
int64_t apogee_callback(alarm_id_t id, void* user_data);
int64_t coast_callback(alarm_id_t id, void* user_data);
void recovery_callback(uint gpio, uint32_t event_mask);

bool timer_callback(repeating_timer_t *rt);
float get_altitude();

volatile float altitude = 0.0f;
volatile float velocity = 0.0f;
volatile state_t state = PAD;
volatile float threshold_altitude = 30.0f;
volatile float threshold_velocity = 30.0f;

int main() {
    stdio_init_all();

    repeating_timer_t timer;
    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);


    alarm_pool_init_default();

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

    // Below configures the interrupt for the first transition from PAD to BOOST
    // Initial Reading
    altitude = get_altitude();
    threshold_altitude += altitude; // 30 meters above ground

    // Select pressure target MSB register(0x16)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x16;
    config[1] = (uint16_t) (((uint32_t)(threshold_altitude) & 0xFF00) >> 16);
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select pressure target LSB register(0x17)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x17;
    config[1] = (uint16_t) (((uint32_t)(threshold_altitude) & 0x00FF));
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    config[0] = 0x29;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    config[0] = 0x2A;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_EDGE_RISE, true, pad_callback);
    // End of configuration of interrupt for first transition from PAD to BOOST

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
    altitude = get_altitude();
    printf("Altitude: %4.2f\n", altitude);
    velocity = ((altitude - prev_altitude) / 0.01f);
    printf("Velocity: %4.2f\n", velocity);
    prev_altitude = altitude;
    absolute_time_t now = get_absolute_time();
    int64_t time_delta = absolute_time_diff_us(last, now);
    printf("Time Delta: %" PRIi64"\n", time_delta);
    return true;
}

void pad_callback(uint gpio, uint32_t event_mask) {

    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_EDGE_RISE, false, pad_callback);
    uint8_t config[2] = {0};
    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    config[0] = 0x29;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    config[0] = 0x2A;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    state = BOOST;
    // start motor burn timer with this function as callback
    add_alarm_in_ms(MOTOR_BURN_TIME, boost_callback, NULL, false);
}

int64_t boost_callback(alarm_id_t id, void* user_data) {
    // Configure accelerometer and/or altimeter to generate interrupt
    // for when velocity is negative with this function as callback to
    // transition to APOGEE
    add_alarm_in_ms(1000, coast_callback, NULL, false);
    state = COAST;
    return 0;
}

int64_t coast_callback(alarm_id_t id, void* user_data) {
    // Want to somehow immediately transition to RECOVERY from APOGEE (extremely short timer?)
    if (velocity < 0.0f) {
        add_alarm_in_ms(1, apogee_callback, NULL, false);
        state = APOGEE;
    } else {
        add_alarm_in_ms(1000, coast_callback, NULL, false);
    }
    return 0;
}

int64_t apogee_callback(alarm_id_t id, void* user_data) {
    // Set altimeter interrupt to occur for when rocket touches back to the ground

    uint8_t config[2] = {0};
    // Select pressure target MSB register(0x16)
    // Set altitude target to 10 meters above ground altitude
    float ground_altitude = threshold_altitude - 20.0f;
    config[0] = 0x16;
    config[1] = (uint16_t) (((uint32_t)(ground_altitude) & 0xFF00) >> 16);
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select pressure target LSB register(0x17)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x17;
    config[1] = (uint16_t) (((uint32_t)(ground_altitude) & 0x00FF));
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    config[0] = 0x29;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    config[0] = 0x2A;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ADDR, config, 2, false);

    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_EDGE_RISE, true, recovery_callback);
    state = RECOVERY;
    return 0;
}

void recovery_callback(uint gpio, uint32_t event_mask) {
    // Essentially just a signal to stop logging data
    state = END;
}

float get_altitude() {
    uint8_t reg[1] = {0};
    i2c_write_blocking(i2c_default, ADDR, reg, 1, false);
    uint8_t data[4] = {0};
    i2c_read_blocking(i2c_default, ADDR, data, 4, false);
    // Exactly how MPL3115A2 datasheet says to retrieve altitude
    uint32_t temp_alt = (data[1] << 24) | (data[2] << 16) | (data[3] << 8);
    float altitude = temp_alt / 65536.0f;
    return altitude;
}
