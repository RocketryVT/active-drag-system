#include "bno055.hpp"

/// @link [Pico BNO055 Example](https://learnembeddedsystems.co.uk/bno005-i2c-example-code)

bno055::bno055() {
    bno055_address = BNO055_ADDRESS_A;
    _sensorID = BNO055_ID;
    default_mode = OPERATION_MODE_NDOF;
}

void bno055::init_bno055() {
    uint8_t chip_id_addr = BNO055_CHIP_ID_ADDR;
    uint8_t id[1];
    i2c_write_blocking(i2c_default, bno055_address, &chip_id_addr, 1, false);
    i2c_read_blocking(i2c_default, bno055_address, id, 1, false);
    if (!id[0] == _sensorID) {
        printf("BNO055 not detected\n");
    }

    // Use internal oscillator
    uint8_t data[2];
    data[0] = BNO055_SYS_TRIGGER_ADDR;
    data[1] = 0x40; // Set to use internal oscillator
    i2c_write_blocking(i2c_default, bno055_address, data, 2, true);

    // Reset all interrupt status bits
    data[0] = BNO055_SYS_TRIGGER_ADDR;
    data[1] = 0x01; // Reset interrupt statu
    // 0x05 = Reset system
    i2c_write_blocking(i2c_default, bno055_address, data, 2, true);

    // Set to normal power mode
    data[0] = BNO055_PWR_MODE_ADDR;
    data[1] = 0x00; // Normal power mode
    i2c_write_blocking(i2c_default, bno055_address, data, 2, true);
    sleep_ms(50); // Wait 50ms for the sensor to switch to normal power mode

    // Page 25 of the datasheet
    // Default Axis Config
    data[0] = BNO055_AXIS_MAP_CONFIG_ADDR;
    data[1] = 0x24; // P1=Z, P2=Y, P3=X
    i2c_write_blocking(i2c_default, bno055_address, data, 2, true);

    // Default Axis Sign
    data[0] = BNO055_AXIS_MAP_SIGN_ADDR;
    data[1] = 0x00; // P1=Positive, P2=Positive, P3=Positive
    i2c_write_blocking(i2c_default, bno055_address, data, 2, true);

    // Set units to m/s^2
    data[0] = BNO055_UNIT_SEL_ADDR;
    data[1] = 0x00; // Windows, Celsius, Degrees, DPS, m/s^2
    i2c_write_blocking(i2c_default, bno055_address, data, 2, true);
    sleep_ms(30);

    //The default operation mode after power-on is CONFIGMODE
    // Set mode to NDOF
    // Takes 7ms to switch from CONFIG mode; see page 21 on datasheet (3.3)
    data[0] = BNO055_OPR_MODE_ADDR;
    data[1] = default_mode; // NDOF
    i2c_write_blocking(i2c_default, bno055_address, data, 2, false);
    sleep_ms(100);

    
}

void bno055::read_lin_accel(volatile LinearAcceleration& linear_acceleration) {
    uint8_t accel[6];
    uint8_t lin_accel_reg = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR;
    i2c_write_blocking(i2c_default, bno055_address, &lin_accel_reg, 1, true);
    i2c_read_blocking(i2c_default, bno055_address, accel, 6, false);
    int16_t x, y, z;
    x = y = z = 0;
    x = ((int16_t)accel[0]) | (((int16_t)accel[1]) << 8);
    y = ((int16_t)accel[2]) | (((int16_t)accel[3]) << 8);
    z = ((int16_t)accel[4]) | (((int16_t)accel[5]) << 8);
    linear_acceleration.x = ((float)x) / 100.0;
    linear_acceleration.y = ((float)y) / 100.0;
    linear_acceleration.z = ((float)z) / 100.0;
}

void bno055::read_abs_quaternion(volatile ABSQuaternion& abs_quaternion) {
    uint8_t quat[8];
    uint8_t quat_reg = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    i2c_write_blocking(i2c_default, bno055_address, &quat_reg, 1, true);
    i2c_read_blocking(i2c_default, bno055_address, quat, 8, false);
    int16_t w, x, y, z;
    w = x = y = z = 0;
    w = ((int16_t)quat[0]) | (((int16_t)quat[1]) << 8);
    x = ((int16_t)quat[2]) | (((int16_t)quat[3]) << 8);
    y = ((int16_t)quat[4]) | (((int16_t)quat[5]) << 8);
    z = ((int16_t)quat[6]) | (((int16_t)quat[7]) << 8);
    abs_quaternion.w = ((float)w) / 16384.0; // 2^14 LSB
    abs_quaternion.x = ((float)x) / 16384.0;
    abs_quaternion.y = ((float)y) / 16384.0;
    abs_quaternion.z = ((float)z) / 16384.0;
}