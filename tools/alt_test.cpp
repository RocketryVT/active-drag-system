#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "pico/time.h"

#include "ms5607.hpp"
#include "heartbeat.hpp"
#include "rp2040_micro.h"

#define DATA_RATE_HZ 200
#define MAX_SCL 400000

bool data_callback(repeating_timer_t *rt);

repeating_timer_t data_timer;

MS5607 altimeter(i2c_default);

int main() {
    stdio_init_all();

    i2c_init(i2c_default, MAX_SCL);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);

    altimeter.initialize();
    heartbeat_initialize(PICO_DEFAULT_LED_PIN);
    // gpio_init(PICO_DEFAULT_LED_PIN);
    // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    // gpio_put(PICO_DEFAULT_LED_PIN, 1);

    add_repeating_timer_us(-1000000 / DATA_RATE_HZ,  &data_callback, NULL, &data_timer);

    while (1) {
        sleep_ms(500);
        printf("\n\nAltitude: %4.2f\nPressure: %4.2f\nTemperature: %4.2f\n", (float) altimeter.get_altitude() / ALTITUDE_SCALE_F, (float) altimeter.get_pressure() / PRESSURE_SCALE_F, (float) altimeter.get_temperature() / TEMPERATURE_SCALE_F);
    }
}

bool data_callback(repeating_timer_t *rt) {
    altimeter.ms5607_start_sample();
    return true;
}
