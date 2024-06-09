#include "imu.hpp"

imu::imu(i2c_inst_t* inst, uint8_t addr, uint8_t id, imu_opmode_t mode) {
    this->inst = inst;
    this->addr = addr;
    this->id = id;
    this->mode = mode;
}

void imu::reset() {
    this->buffer[0] = SYS_TRIGGER;
    this->buffer[1] = 0x20; // Reset system
    i2c_write_blocking(this->inst, this->addr, buffer, 2, true);
    sleep_ms(1000); // Wait 650ms for the sensor to reset
}

void imu::initialize() {
    sleep_ms(1000); // Wait 650ms for the sensor to reset

    uint8_t chip_id_addr = CHIP_ID;
    uint8_t read_id = 0x00;
    while (read_id != this->id) {
        i2c_write_blocking(this->inst, this->addr, &chip_id_addr, 1, false);
        i2c_read_blocking(this->inst, this->addr, &read_id, 1, false);
        sleep_ms(100);
    }

    // Use internal oscillator
    this->buffer[0] = SYS_TRIGGER;
    this->buffer[1] = 0x40; // Set to use internal oscillator
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
    sleep_ms(50);

    // Reset all interrupt status bits
    this->buffer[0] = SYS_TRIGGER;
    this->buffer[1] = 0x01; // Reset interrupt status
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
    sleep_ms(50);

    // Set to normal power mode
    this->buffer[0] = POWER_MODE;
    this->buffer[1] = 0x00; // Normal power mode
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
    sleep_ms(50);

    // Default Axis Config
    this->buffer[0] = AXIS_MAP_CONFIG;
    this->buffer[1] = 0x24; // P1=Z, P2=Y, P3=X
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
    sleep_ms(50);

    // Default Axis Sign
    this->buffer[0] = AXIS_MAP_SIGN;
    this->buffer[1] = 0x00; // P1=Positive, P2=Positive, P3=Positive
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
    sleep_ms(50);

    // Set units to m/s^2
    this->buffer[0] = UNIT_SELECTION;
    this->buffer[1] = 0x00; // Windows, Celsius, Degrees, DPS, m/s^2
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
    sleep_ms(50);

    // The default operation mode after power-on is CONFIG
    // Set to desired mode
    this->buffer[0] = OPERATION_MODE;
    this->buffer[1] = this->mode; // NDOF
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, false);
    sleep_ms(100);
}

void imu::calibration_status(calibration_status_t* status) {
    read_register(CALIBRATION_STATUS, 1);
    status->mag = ((this->buffer[0] & 0b00000011) >> 0);
    status->accel = ((this->buffer[0] & 0b00001100) >> 2);
    status->gyro = ((this->buffer[0] & 0b00110000) >> 4);
    status->sys = ((this->buffer[0] & 0b11000000) >> 6);
}

void imu::linear_acceleration(Eigen::Vector3f& vec) {
    read_register(LINEAR_ACCELERATION_X_LSB, 6);
    int16_t x, y, z;
    x = y = z = 0;
    x = ((int16_t)this->buffer[0]) | (((int16_t)this->buffer[1]) << 8);
    y = ((int16_t)this->buffer[2]) | (((int16_t)this->buffer[3]) << 8);
    z = ((int16_t)this->buffer[4]) | (((int16_t)this->buffer[5]) << 8);
    vec(0) = ((float)x) / 100.0;
    vec(1) = ((float)y) / 100.0;
    vec(2) = ((float)z) / 100.0;
}

void imu::quaternion(Eigen::Vector4f& vec) {
    read_register(QUATERNION_W_LSB, 8);
    int16_t w, x, y, z;
    w = x = y = z = 0;
    w = ((int16_t)this->buffer[0]) | (((int16_t)this->buffer[1]) << 8);
    x = ((int16_t)this->buffer[2]) | (((int16_t)this->buffer[3]) << 8);
    y = ((int16_t)this->buffer[4]) | (((int16_t)this->buffer[5]) << 8);
    z = ((int16_t)this->buffer[6]) | (((int16_t)this->buffer[7]) << 8);
    vec(0) = ((float)w) / 16384.0;
    vec(1) = ((float)x) / 16384.0;
    vec(2) = ((float)y) / 16384.0;
    vec(3) = ((float)z) / 16384.0;
}

void imu::read_register(uint8_t reg, size_t len) {
    i2c_write_blocking(this->inst, this->addr, &reg, 1, true);
    i2c_read_blocking(this->inst, this->addr, this->buffer, len, false);
}

