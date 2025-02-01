#include <stdio.h>

#include "hardware/gpio.h"
#include "boards/pico.h"
#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/types.h"
#include <inttypes.h>

#define ALT_ADDR 0x77
#define MAX_SCL 400000

int64_t pressure_read_callback(alarm_id_t id, void* user_data);
int64_t temperature_read_callback(alarm_id_t id, void* user_data);

uint32_t pressure_adc = 0;
uint32_t temperature_adc = 0;

int main() {
    stdio_init_all();

    i2c_init(i2c0, MAX_SCL);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_function(21, GPIO_FUNC_I2C);
    gpio_set_function(20, GPIO_FUNC_I2C);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    alarm_pool_init_default();

    getchar();

    uint8_t cmd = 0x1E;
    uint8_t prom_raw[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
    printf("resetting the baro sensor\n");
    sleep_ms(500);

    uint8_t prom_cmd_start = 0xA0;

    printf("getting prom data\n");
    for (uint8_t i = 1; i < 7; i++) {
        sleep_ms(100);
        cmd = (prom_cmd_start | (i << 1));
        i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
        i2c_read_blocking(i2c0, ALT_ADDR, (uint8_t *)(prom_raw + (i-1)*2), 2, false);
    }
    uint16_t prom[6] = {static_cast<uint16_t>((((uint16_t) prom_raw[0]) << 8) | ((uint16_t) prom_raw[1])),
                        static_cast<uint16_t>((((uint16_t) prom_raw[2]) << 8) | ((uint16_t) prom_raw[3])),
                        static_cast<uint16_t>((((uint16_t) prom_raw[4]) << 8) | ((uint16_t) prom_raw[5])),
                        static_cast<uint16_t>((((uint16_t) prom_raw[6]) << 8) | ((uint16_t) prom_raw[7])),
                        static_cast<uint16_t>((((uint16_t) prom_raw[8]) << 8) | ((uint16_t) prom_raw[9])),
                        static_cast<uint16_t>((((uint16_t) prom_raw[10]) << 8) | ((uint16_t) prom_raw[11]))};
    cmd = 0x40;

    getchar();

    for (uint8_t i = 0; i < 6; i++) {
        printf("%04X ", prom[i]);
    }

    printf("\n");

    while (1) {

        getchar();
        printf("requesting adc conversions\n");

        i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);

        add_alarm_in_us(500, &pressure_read_callback, NULL, true);
        
        sleep_ms(100);

        absolute_time_t end_time;
        absolute_time_t start_time = get_absolute_time();

        int32_t dT = temperature_adc - (((uint32_t) prom[4]) << 8);
        int32_t actual_temp = 2000 + ( ( ( (int64_t) dT) * ( (int64_t) prom[5]) ) >> 23);
        int64_t OFF = ( ( (int64_t) prom[1]) << 17) + ( ( ((int64_t) prom[3]) * ( (int64_t) dT)) >> 6);
        int64_t SENS = ( ( (int64_t) prom[0]) << 16) + (( ( (int64_t) prom[2]) * ((int64_t) dT)) >> 7);
        int32_t actual_pres = (int32_t) ((((((int64_t) pressure_adc) * SENS) >> 21) - OFF) >> 15);

        end_time = get_absolute_time();

        int64_t microseconds =  absolute_time_diff_us(start_time, end_time);

        printf("Pressure: %4.2f\nTemperature: %4.2f\nTime to Convert: %" PRIi64 " us\n", ((float) (actual_pres)) / 100.0f, ((float) (actual_temp)) / 100.0f, microseconds);

        pressure_adc = 0;
        temperature_adc = 0;

    }

}

int64_t pressure_read_callback(alarm_id_t id, void* user_data) {
    uint8_t cmd = 0;
    uint8_t buffer[3] = {0, 0, 0};
    i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
    i2c_read_blocking(i2c0, ALT_ADDR, (uint8_t *)(buffer), 3, false);

    pressure_adc = (((uint32_t) buffer[0]) << 16) | (((uint32_t) buffer[1]) << 8) | ((uint32_t) buffer[0]);

    cmd = 0x50;
    i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
    add_alarm_in_us(500, &temperature_read_callback, NULL, true);
    return 0;
}

int64_t temperature_read_callback(alarm_id_t id, void* user_data) {
    uint8_t cmd = 0;
    uint8_t buffer[3] = {0, 0, 0};
    i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
    i2c_read_blocking(i2c0, ALT_ADDR, (uint8_t *)(buffer), 3, false);

    temperature_adc = (((uint32_t) buffer[0]) << 16) | (((uint32_t) buffer[1]) << 8) | ((uint32_t) buffer[0]);
    return 0;
}
