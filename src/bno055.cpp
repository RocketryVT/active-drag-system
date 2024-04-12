#include "bno055.hpp"

bno055::bno055() {
    deviceAddress = BNO055_ADDRESS_A;
    _sensorID = BNO055_ID;
    default_mode = OPERATION_MODE_NDOF;
}

void bno055::init_bno055() {
    uint8_t id;
    i2c_read_blocking(i2c_default, deviceAddress, &id, 1, false);
    if (!id == _sensorID) {
        printf("BNO055 not detected\n");
    }
    // setModeHard(OPERATION_MODE_CONFIG);
}

void bno055::read_lin_accel(volatile LinearAcceleration& linear_acceleration) {
    uint8_t mode = static_cast<uint8_t>(default_mode);
    i2c_write_blocking(i2c_default, BNO055_ADDRESS_A, &mode, 1, false);

    uint8_t buffer[6];
    i2c_read_blocking(i2c_default, BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6, false);
    int16_t x, y, z;
    x = y = z = 0;
    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
    linear_acceleration.x = ((double)x) / 100.0;
    linear_acceleration.y = ((double)y) / 100.0;
    linear_acceleration.z = ((double)z) / 100.0;
}