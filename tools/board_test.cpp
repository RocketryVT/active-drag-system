#include <cstdint>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "boards/pico.h"
#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico/types.h"
#include <inttypes.h>

#define ALT_ADDR 0x77
#define IMU_ADDR 0x68
#define ACC_ADDR 0x1D
#define MAG_ADDR

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


/*IIM-42653
	//Enable 40kHz clock output on GPIO 21
	//0x06 is for clock source, 3125 is 125MHz to 40kHz divider
	clock_gpio_init(23, 0x6, 3125);	//TODO: Store these constants somewhere?
    uint8_t imu_reg = 0x4F;
    uint8_t imu_val = 0x0F;
    uint8_t imu_act[2] = {imu_reg, imu_val};
    i2c_write_blocking(i2c0, IMU_ADDR, imu_act, 2, true);
    imu_reg = 0x50;
    imu_act[0] = imu_reg;
    i2c_write_blocking(i2c0, IMU_ADDR, imu_act, 2, true);
    imu_reg = 0x4E;
    imu_val = 0b00101111;
    imu_act[0] = imu_reg;
    imu_act[1] = imu_val;
    i2c_write_blocking(i2c0, IMU_ADDR, imu_act, 2, true);
    sleep_ms(100);
    uint8_t buf[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
IIM-42653*/

//    alarm_pool_init_default();

    getchar();

    uint8_t acc_act[2] = {0x2C, 0x0D};
    i2c_write_blocking(i2c0, ACC_ADDR, acc_act, 2, true);
    acc_act[0] = 0x2D;
    acc_act[1] = 0x08;
    i2c_write_blocking(i2c0, ACC_ADDR, acc_act, 2, true);
    acc_act[0] = 0x31;
    acc_act[1] = 0b00001011;
    i2c_write_blocking(i2c0, ACC_ADDR, acc_act, 2, true);
    sleep_ms(10);
/*MS5607
//    uint8_t cmd = 0x1E;
//    uint8_t prom_raw[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//
//    i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
//    printf("resetting the baro sensor\n");
//    sleep_ms(500);
//
//    uint8_t prom_cmd_start = 0xA0;
//
//    printf("getting prom data\n");
//    for (uint8_t i = 1; i < 7; i++) {
//        sleep_ms(100);
//        cmd = (prom_cmd_start | (i << 1));
//        i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
//        i2c_read_blocking(i2c0, ALT_ADDR, (uint8_t *)(prom_raw + (i-1)*2), 2, false);
//    }
//    uint16_t prom[6] = {static_cast<uint16_t>((((uint16_t) prom_raw[0]) << 8) | ((uint16_t) prom_raw[1])),
//                        static_cast<uint16_t>((((uint16_t) prom_raw[2]) << 8) | ((uint16_t) prom_raw[3])),
//                        static_cast<uint16_t>((((uint16_t) prom_raw[4]) << 8) | ((uint16_t) prom_raw[5])),
//                        static_cast<uint16_t>((((uint16_t) prom_raw[6]) << 8) | ((uint16_t) prom_raw[7])),
//                        static_cast<uint16_t>((((uint16_t) prom_raw[8]) << 8) | ((uint16_t) prom_raw[9])),
//                        static_cast<uint16_t>((((uint16_t) prom_raw[10]) << 8) | ((uint16_t) prom_raw[11]))};
//    cmd = 0x40;
//
//    getchar();
//
//    for (uint8_t i = 0; i < 6; i++) {
//        printf("%04X ", prom[i]);
//    }
//
//    printf("\n");
MS5607 */

/* IIM-42653
    float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
    imu_reg = 0x1F;
IIM-42653*/

    absolute_time_t end_time, start_time;
    float accel_high_x, accel_high_y, accel_high_z;
    uint8_t acc_reg = 0x32;
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    while (1) {
        start_time = get_absolute_time();
        i2c_write_blocking(i2c0, ACC_ADDR, &acc_reg, 1, true);
        i2c_read_blocking(i2c0, ACC_ADDR, buf, 6, false);

        int16_t x, y, z;
        x = y = z = 0;

        x = ((int16_t) buf[0]) | ((int16_t) buf[1] << 8);

        y = ((int16_t) buf[2]) | ((int16_t) buf[3] << 8);

        z = ((int16_t) buf[4]) | ((int16_t) buf[5] << 8);

        accel_high_x = ((float) x) / 20.5;

        accel_high_y = ((float) y) / 20.5;

        accel_high_z = ((float) z) / 20.5;

        end_time = get_absolute_time();

        int64_t micros = absolute_time_diff_us(start_time, end_time);

        printf("accel_x: %4.2f, accel_y: %4.2f, accel_z: %4.2f, time: %" PRIi64 "\n",
                accel_high_x, accel_high_y, accel_high_z, micros);

        sleep_ms(100);
/* IIM-42653
//        start_time = get_absolute_time();
//        i2c_write_blocking(i2c0, IMU_ADDR, &imu_reg, 1, true);
//        i2c_read_blocking(i2c0, IMU_ADDR, buf, 12, false);
//
//        int16_t x, y, z;
//        x = y = z = 0;
//
//        x = ((int16_t)buf[1]) | (((int16_t)buf[0]) << 8);
//        y = ((int16_t)buf[3]) | (((int16_t)buf[2]) << 8);
//        z = ((int16_t)buf[5]) | (((int16_t)buf[4]) << 8);
//
//        accel_x = ((float) x) / 1024.0f;
//        accel_y = ((float) y) / 1024.0f;
//        accel_z = ((float) z) / 1024.0f;
//
//        x = y = z = 0;
//
//        x = ((int16_t)buf[7]) | (((int16_t)buf[6]) << 8);
//        y = ((int16_t)buf[9]) | (((int16_t)buf[8]) << 8);
//        z = ((int16_t)buf[11]) | (((int16_t)buf[10]) << 8);
//
//        gyro_x = ((float) x) / 8.2f;
//        gyro_y = ((float) y) / 8.2f;
//        gyro_z = ((float) z) / 8.2f;
//
//        for (uint8_t i = 0; i < 12; i++) {
//            buf[i] = 0;
//        }
//        end_time = get_absolute_time();
//
//        int64_t micros = absolute_time_diff_us(start_time, end_time);
//
//        printf("accel_x: %4.2f, accel_y: %4.2f, accel_z: %4.2f, gyro_x: %4.2f, gyro_y: %4.2f, gyro_z: %4.2f, time: %" PRIi64 "\n",
//                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, micros);
//        sleep_ms(100);
IIM-42653 */

//        i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
//
//        add_alarm_in_us(500, &pressure_read_callback, NULL, true);
//
//
//
//        while (!temperature_available) {}
//        
//        int32_t dT = temperature_adc - (((uint32_t) prom[4]) << 8);
//        int32_t actual_temp = 2000 + ( ( ( (int64_t) dT) * ( (int64_t) prom[5]) ) >> 23);
//
//        while (!pressure_available) {}
//
//        int64_t OFF = ( ( (int64_t) prom[1]) << 17) + ( ( ((int64_t) prom[3]) * ( (int64_t) dT)) >> 6);
//        int64_t SENS = ( ( (int64_t) prom[0]) << 16) + (( ( (int64_t) prom[2]) * ((int64_t) dT)) >> 7);
//        int32_t actual_pres = (int32_t) ((((((int64_t) pressure_adc) * SENS) >> 21) - OFF) >> 15);

        // printf("Pressure: %4.2f\nTemperature: %4.2f\nTime to Convert: %" PRIi64 " us\n", ((float) (actual_pres)) / 100.0f, ((float) (actual_temp)) / 100.0f, microseconds);
    }

}

// int64_t pressure_read_callback(alarm_id_t id, void* user_data) {
//     uint8_t cmd = 0;
//     uint8_t buffer[3] = {0, 0, 0};
//     i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
//     i2c_read_blocking(i2c0, ALT_ADDR, (uint8_t *)(buffer), 3, false);
// 
//     pressure_adc = (((uint32_t) buffer[0]) << 16) | (((uint32_t) buffer[1]) << 8) | ((uint32_t) buffer[0]);
// 
//     cmd = 0x50;
//     i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
//     add_alarm_in_us(500, &temperature_read_callback, NULL, true);
//     return 0;
// }
// 
// int64_t temperature_read_callback(alarm_id_t id, void* user_data) {
//     uint8_t cmd = 0;
//     uint8_t buffer[3] = {0, 0, 0};
//     i2c_write_blocking(i2c0, ALT_ADDR, &cmd, 1, true);
//     i2c_read_blocking(i2c0, ALT_ADDR, (uint8_t *)(buffer), 3, false);
// 
//     temperature_adc = (((uint32_t) buffer[0]) << 16) | (((uint32_t) buffer[1]) << 8) | ((uint32_t) buffer[0]);
//     return 0;
// }
