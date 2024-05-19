#include <stdio.h>

#include "hardware/gpio.h"
#include "boards/pico_w.h"
#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "pico/time.h"

#define ALT_ADDR 0x60
#define MAX_SCL 400000
#define DATA_RATE_HZ 15

float altitude = 0.0f;
float get_altitude();

int main() {
    stdio_init_all();

    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    uint8_t config[2] = {0};

    // Select control register(0x26)
    // Active mode, OSR = 16, altimeter mode(0xB8)
    config[0] = 0x26;
    config[1] = 0xB9;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);
    sleep_ms(1500);

    while (1) {
        sleep_ms(1000);
        altitude = get_altitude();
        printf("Altitude: %4.2f\n", altitude);
    }
}

float get_altitude() {
    uint8_t reg = 0x01;
    uint8_t data[5];
    i2c_write_blocking(i2c_default, ALT_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, ALT_ADDR, data, 5, false);
    // Exactly how MPL3115A2 datasheet says to retrieve altitude
    float altitude = (float) ((int16_t) ((data[0] << 8) | data[1])) + (float) (data[2] >> 4) * 0.0625;
    return altitude;
}

