#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <Eigen/Geometry>

#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define MAX_SCL 400000

#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_OPR_MODE_CONFIG 0x00
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_ADDRESS 0x28
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_CHIP_ID 0xA0
#define BNO055_OPR_MODE_NDOF 0x0C
#define BNO055_CALIB_STAT_ADDR 0x35
#define ACCEL_OFFSET_X_LSB_ADDR 0x55
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define UNIT_SELECTION 0x3B

void get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);

int main() {
    stdio_init_all();

    getchar();

    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    uint8_t buf[2] = {BNO055_CHIP_ID_ADDR};

    uint8_t id = 0x00;
    sleep_ms(1000);
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 1, false);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS, &id, 1, false);
    while (id != BNO055_CHIP_ID) {
        i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 1, false);
        i2c_read_blocking(i2c_default, BNO055_ADDRESS, &id, 1, false);
        printf("Id not correct!, seeing: %" PRIu8 "\n", id);
        sleep_ms(10);
    }

    buf[0] = BNO055_OPR_MODE_ADDR;
    buf[1] = BNO055_OPR_MODE_CONFIG;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);

    buf[0] = BNO055_SYS_TRIGGER_ADDR;
    buf[1] = 0x20; // RESET
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);
    sleep_ms(30);

    buf[0] = BNO055_CHIP_ID_ADDR;
    id = 0x00;
    while (id != BNO055_CHIP_ID) {
        i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 1, false);
        i2c_read_blocking(i2c_default, BNO055_ADDRESS, &id, 1, false);
        printf("Id not correct!, seeing: %" PRIu8 "\n", id);
        sleep_ms(10);
    }

    buf[0] = BNO055_SYS_TRIGGER_ADDR;
    buf[1] = 0x00; // RESET
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);
    sleep_ms(30);
    
    // Set units to m/s^2
    buf[0] = UNIT_SELECTION;
    buf[1] = 0x00; // Windows, Celsius, Degrees, DPS, m/s^2
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);
    sleep_ms(50);

    buf[0] = BNO055_OPR_MODE_ADDR;
    buf[1] = BNO055_OPR_MODE_NDOF;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);

    uint8_t gyro = 0x00, accel = 0x00, mag = 0x00;

    printf("Magnetometer: Perform the figure-eight calibration dance.\n");
    while (mag != 3) {
        // Calibration Dance Step One: Magnetometer
        //   Move sensor away from magnetic interference or shields
        //   Perform the figure-eight until calibrated
        get_calibration(NULL, NULL, NULL, &mag);
        printf("Mag Calib Status: %3.0f\n", (100 / 3 * mag));
        sleep_ms(1000);
    }
    printf("... CALIBRATED\n");
    sleep_ms(1000);

    printf("Accelerometer: Perform the six-step calibration dance.\n");
    while (accel != 3) {
        // Calibration Dance Step Two: Accelerometer
        //   Place sensor board into six stable positions for a few seconds each:
        //    1) x-axis right, y-axis up,    z-axis away
        //    2) x-axis up,    y-axis left,  z-axis away
        //    3) x-axis left,  y-axis down,  z-axis away
        //    4) x-axis down,  y-axis right, z-axis away
        //    5) x-axis left,  y-axis right, z-axis up
        //    6) x-axis right, y-axis left,  z-axis down
        //   Repeat the steps until calibrated
        get_calibration(NULL, NULL, &accel, NULL);
        printf("Accel Calib Status: %3.0f\n", (100 / 3 * accel));
        sleep_ms(1000);
    }
    printf("... CALIBRATED\n");
    sleep_ms(1000);

    printf("Gyroscope: Perform the hold-in-place calibration dance.\n");
    while (gyro != 3) {
        // Calibration Dance Step Three: Gyroscope
        //  Place sensor in any stable position for a few seconds
        //  (Accelerometer calibration may also calibrate the gyro)
        get_calibration(NULL, &gyro, NULL, NULL);
        printf("Gyro Calib Status: %3.0f\n", (100 / 3 * gyro));
        sleep_ms(1000);
    }
    printf("... CALIBRATED\n");
    sleep_ms(1000);
    printf("CALIBRATION COMPLETED\n");

    // Get Sensor Offsets
    buf[0] = BNO055_OPR_MODE_ADDR;
    buf[1] = BNO055_OPR_MODE_CONFIG;
    uint8_t sensor_offsets[22];
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);
    sleep_ms(30);

    buf[0] = ACCEL_OFFSET_X_LSB_ADDR;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 1, false);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS, sensor_offsets, 18, false);
    for (uint8_t i = 0; i < 18; i++) {
        printf("sensor_offsets[%" PRIu8 "] = 0x%" PRIx8 ";\r\n", i + 1, sensor_offsets[i]);
    }
    sleep_ms(5000);

    buf[0] = BNO055_OPR_MODE_ADDR;
    buf[1] = BNO055_OPR_MODE_NDOF;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 2, false);
    sleep_ms(5000);

    getchar();

    uint8_t lin_accel[6];
    uint8_t quat[8];
    float accel_x, accel_y, accel_z;
    float abs_lin_accel_x, abs_lin_accel_y, abs_lin_accel_z;
    float abs_quaternion_w, abs_quaternion_x, abs_quaternion_y, abs_quaternion_z;
    while (1) {
        uint8_t lin_accel_reg = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
        i2c_write_blocking(i2c_default, BNO055_ADDRESS, &lin_accel_reg, 1, true);
        i2c_read_blocking(i2c_default, BNO055_ADDRESS, lin_accel, 6, false);
        int16_t x, y, z;
        x = y = z = 0;
        x = ((int16_t)lin_accel[0]) | (((int16_t)lin_accel[1]) << 8);
        y = ((int16_t)lin_accel[2]) | (((int16_t)lin_accel[3]) << 8);
        z = ((int16_t)lin_accel[4]) | (((int16_t)lin_accel[5]) << 8);
        accel_x = ((float)x) / 100.0;
        accel_y = ((float)y) / 100.0;
        accel_z = ((float)z) / 100.0;

        uint8_t quat_reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;
        i2c_write_blocking(i2c_default, BNO055_ADDRESS, &quat_reg, 1, true);
        i2c_read_blocking(i2c_default, BNO055_ADDRESS, quat, 8, false);
        int16_t w;
        w = x = y = z = 0;
        w = ((int16_t)quat[0]) | (((int16_t)quat[1]) << 8);
        x = ((int16_t)quat[2]) | (((int16_t)quat[3]) << 8);
        y = ((int16_t)quat[4]) | (((int16_t)quat[5]) << 8);
        z = ((int16_t)quat[6]) | (((int16_t)quat[7]) << 8);
        abs_quaternion_w = ((float)w) / 16384.0; // 2^14 LSB
        abs_quaternion_x = ((float)x) / 16384.0;
        abs_quaternion_y = ((float)y) / 16384.0;
        abs_quaternion_z = ((float)z) / 16384.0;

        Eigen::Quaternion<float> q;
        q.w() = abs_quaternion_w;
        q.x() = abs_quaternion_x;
        q.y() = abs_quaternion_y;
        q.z() = abs_quaternion_z;
        // q.normalize();
        Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();
        Eigen::Vector3f lin_accel;
        abs_lin_accel_x = accel_x* rotation_matrix(0, 0) + accel_y * rotation_matrix(0, 1) + accel_z* rotation_matrix(0, 2);
        abs_lin_accel_y = accel_x * rotation_matrix(1, 0) + accel_y * rotation_matrix(1, 1) + accel_z * rotation_matrix(1, 2);
        abs_lin_accel_z = -1.0f * (accel_x * rotation_matrix(2, 0) + accel_y * rotation_matrix(2, 1) + accel_z * rotation_matrix(2, 2));

        printf("Acceleration Vector: %4.2f, %4.2f, %4.2f\n", accel_x, accel_y, accel_z);
        printf("Abs Acceleration Vector: %4.2f, %4.2f, %4.2f\n", abs_lin_accel_x, abs_lin_accel_y, abs_lin_accel_z);
        printf("Quaternion: %4.2f, %4.2f, %4.2f, %4.2f\n\n\n", abs_quaternion_w, abs_quaternion_x, abs_quaternion_y, abs_quaternion_z);
        sleep_ms(1000);
    }

    return 0;
}

void get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t buf[1] = {BNO055_CALIB_STAT_ADDR};
    uint8_t cal_data = 0x00;
    i2c_write_blocking(i2c_default, BNO055_ADDRESS, buf, 1, false);
    i2c_read_blocking(i2c_default, BNO055_ADDRESS, &cal_data, 1, false);
    if (sys != NULL) {
        *sys = (cal_data >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (cal_data >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (cal_data >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = cal_data & 0x03;
    }
}

