#pragma once

#include <stdint.h>
#include "hardware/i2c.h"

#include <Eigen/Dense>


#define BNO055_NUM_OFFSET_REGISTERS 22


typedef enum {
    CONFIG = 0x00,
    ACCELERATON_ONLY = 0x01,
    MAGNETOMETER_ONLY = 0x02,
    GYROSCOPE_ONLY = 0x03,
    ACCEL_MAG = 0x04,
    ACCEL_GYRO = 0x05,
    MAG_GYRO = 0x06,
    ACCEL_MAG_GYRO = 0x07,
    IMU_PLUS = 0x08,
    COMPASS = 0x09,
    M4G = 0x0A,
    NDOF_FMC_OFF = 0x0B,
    NDOF = 0x0C
} imu_opmode_t;

typedef enum {
    PAGE_ID = 0x07,

    CHIP_ID = 0x00,
    ACCEL_REV_ID = 0x01,
    MAG_REV_ID = 0x02,
    GYRO_REV_ID = 0x03,
    SW_REV_ID_LSB = 0x04,
    SW_REV_ID_MSB = 0x05,
    BL_REV_ID = 0x06,

    ACCELERATION_X_LSB = 0x08,
    ACCELERATION_X_MSB = 0x09,
    ACCELERATION_Y_LSB = 0x0A,
    ACCELERATION_Y_MSB = 0x0B,
    ACCELERATION_Z_LSB = 0x0C,
    ACCELERATION_Z_MSB = 0x0D,

    MAGNETOMETER_X_LSB = 0x0E,
    MAGNETOMETER_X_MSB = 0x0F,
    MAGNETOMETER_Y_LSB = 0x10,
    MAGNETOMETER_Y_MSB = 0x11,
    MAGNETOMETER_Z_LSB = 0x12,
    MAGNETOMETER_Z_MSB = 0x13,

    GYROSCOPE_X_LSB = 0x14,
    GYROSCOPE_X_MSB = 0x15,
    GYROSCOPE_Y_LSB = 0x16,
    GYROSCOPE_Y_MSB = 0x17,
    GYROSCOPE_Z_LSB = 0x18,
    GYROSCOPE_Z_MSB = 0x19,

    EULER_H_LSB = 0x1A,
    EULER_H_MSB = 0x1B,
    EULER_R_LSB = 0x1C,
    EULER_R_MSB = 0x1D,
    EULER_P_LSB = 0x1E,
    EULER_P_MSB = 0x1F,

    QUATERNION_W_LSB = 0x20,
    QUATERNION_W_MSB = 0x21,
    QUATERNION_X_LSB = 0x22,
    QUATERNION_X_MSB = 0x23,
    QUATERNION_Y_LSB = 0x24,
    QUATERNION_Y_MSB = 0x25,
    QUATERNION_Z_LSB = 0x26,
    QUATERNION_Z_MSB = 0x27,

    LINEAR_ACCELERATION_X_LSB = 0x28,
    LINEAR_ACCELERATION_X_MSB = 0x29,
    LINEAR_ACCELERATION_Y_LSB = 0x2A,
    LINEAR_ACCELERATION_Y_MSB = 0x2B,
    LINEAR_ACCELERATION_Z_LSB = 0x2C,
    LINEAR_ACCELERATION_Z_MSB = 0x2D,

    GRAVITY_X_LSB = 0x2E,
    GRAVITY_X_MSB = 0x2F,
    GRAVITY_Y_LSB = 0x30,
    GRAVITY_Y_MSB = 0x31,
    GRAVITY_Z_LSB = 0x32,
    GRAVITY_Z_MSB = 0x33,

    TEMPERATURE = 0x34,

    CALIBRATION_STATUS = 0x35,
    SELF_TEST_RESULT = 0x36,
    INTERRUPT_STATUS = 0x37,

    SYS_CLK_STATUS = 0x38,
    SYS_STATUS = 0x39,
    SYS_ERROR = 0x3A,

    UNIT_SELECTION = 0x3B,

    OPERATION_MODE = 0x3D,
    POWER_MODE = 0x3E,

    SYS_TRIGGER = 0x3F,
    TEMP_SOURCE = 0x40,

    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,

    SIC_MATRIX_0_LSB = 0x43,
    SIC_MATRIX_0_MSB = 0x44,
    SIC_MATRIX_1_LSB = 0x45,
    SIC_MATRIX_1_MSB = 0x46,
    SIC_MATRIX_2_LSB = 0x47,
    SIC_MATRIX_2_MSB = 0x48,
    SIC_MATRIX_3_LSB = 0x49,
    SIC_MATRIX_3_MSB = 0x4A,
    SIC_MATRIX_4_LSB = 0x4B,
    SIC_MATRIX_4_MSB = 0x4C,
    SIC_MATRIX_5_LSB = 0x4D,
    SIC_MATRIX_5_MSB = 0x4E,
    SIC_MATRIX_6_LSB = 0x4F,
    SIC_MATRIX_6_MSB = 0x50,
    SIC_MATRIX_7_LSB = 0x51,
    SIC_MATRIX_7_MSB = 0x52,
    SIC_MATRIX_8_LSB = 0x53,
    SIC_MATRIX_8_MSB = 0x54,

    ACCELERATION_OFFSET_X_LSB = 0x55,
    ACCELERATION_OFFSET_X_MSB = 0x56,
    ACCELERATION_OFFSET_Y_LSB = 0x57,
    ACCELERATION_OFFSET_Y_MSB = 0x58,
    ACCELERATION_OFFSET_Z_LSB = 0x59,
    ACCELERATION_OFFSET_Z_MSB = 0x5A,

    MAGNETOMETER_OFFSET_X_LSB = 0x5B,
    MAGNETOMETER_OFFSET_X_MSB = 0x5C,
    MAGNETOMETER_OFFSET_Y_LSB = 0x5D,
    MAGNETOMETER_OFFSET_Y_MSB = 0x5E,
    MAGNETOMETER_OFFSET_Z_LSB = 0x5F,
    MAGNETOMETER_OFFSET_Z_MSB = 0x60,

    GYROSCOPE_OFFSET_X_LSB = 0x61,
    GYROSCOPE_OFFSET_X_MSB = 0x62,
    GYROSCOPE_OFFSET_Y_LSB = 0x63,
    GYROSCOPE_OFFSET_Y_MSB = 0x64,
    GYROSCOPE_OFFSET_Z_LSB = 0x65,
    GYROSCOPE_OFFSET_Z_MSB = 0x66,

    ACCEL_RADIUS_LSB = 0x67,
    ACCEL_RADIUS_MSB = 0x68,
    MAG_RADIUS_LSB = 0x69,
    MAG_RADIUS_MSB = 0x6A,

    RESET_INTERRUPT = 0x01,

    NO_MOTION_INTERRUPT = 0x00,
    SLOW_NO_MOTION_INTERRUPT = 0x01,
    THRESHOLD_INTERRUPT = 0x02,
} imu_reg_t;

typedef enum {
    NORMAL = 0x00,
    LOW_POWER = 0x01,
    SUSPEND = 0x02
} imu_power_mode_t;

typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} calibration_status_t ;

class imu {
    private:
        i2c_inst_t* inst;
        uint8_t addr;
        uint8_t id;
        imu_opmode_t mode;

        uint8_t buffer[10];
        uint8_t accel_buffer[6];
        uint8_t quat_buffer[8];

        void read_register(uint8_t reg, size_t len, uint8_t* buffer);

    public:
        imu(i2c_inst_t* inst, uint8_t addr, uint8_t id, imu_opmode_t mode);

        void initialize();

        void reset();

        void linear_acceleration(Eigen::Vector3f& vec);

        void quaternion(Eigen::Vector4f& vec);

        void quaternion_euler(Eigen::Vector3f& angles, Eigen::Vector4f& quat);

        void calibration_status(calibration_status_t* status);

        uint32_t expose_acceleration_buffer(uint8_t** buffer);

        uint32_t expose_quaternion_buffer(uint8_t** buffer);
};
