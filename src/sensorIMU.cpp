#include "../include/sensorIMU.hpp"

IMUSensor::IMUSensor(std::string I2C_FILE) {
    this -> I2C_FILE = I2C_FILE;
}

bool IMUSensor::init(void* data) {

    //I2C_File passed on object creation, stored in sensorI2C parent
    setupI2C(I2C_FILE);
    
    //In the adafruit code there's a big step of waiting for timeout and connection stuff for up to a full second
    //I don't do that here because the BBB takes like 17 years to boot so we'll just hope it goes faster than that

    //Sanity check for factory device ID 
    uint8_t id = readSingleRegister(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
        fprintf(stderr, "DEVICE ID DID NOT PASS SANITY CHECK FOR BNO IMU!");
        return false;
    }

    //Set default operating mode of IMU into config from startup (will be set properly after config phase)
    setModeHard(OPERATION_MODE_CONFIG);

    //Writes 1 to the system reset bit in the trigger register
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x20);
    //Wait for reset to complete by doing sanity check again
    while (readSingleRegister(BNO055_CHIP_ID_ADDR) != BNO055_ID);

    //Set power mode for sensor
    writeRegister(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);

    //Sensor chip uses two "pages" to multiplex register values
    //Page 0 contains the sensor data (not configuration), which is what we want
    writeRegister(BNO055_PAGE_ID_ADDR, 0);

    //Genuinely no idea why Adafruit does this, ensuring all triggers are off before mode config I guess
    writeRegister(BNO055_SYS_TRIGGER_ADDR, 0x0);

    setModeTemp(default_mode);

    return true;
}

//Sets mode so it can be undone for temporary changes, like operation setting
void IMUSensor::setModeTemp(adafruit_bno055_opmode_t mode) {
    currentMode = mode;
    writeRegister(BNO055_OPR_MODE_ADDR, currentMode);
}

//Sets mode *AND* internal state variable
void IMUSensor::setModeTemp(adafruit_bno055_opmode_t mode) {
    writeRegister(BNO055_OPR_MODE_ADDR, currentMode);
}

adafruit_bno055_opmode_t IMUSensor::getMode() {
    return (adafruit_bno055_opmode_t)readSingleRegister(BNO055_OPR_MODE_ADDR);
}

imu::Vector<3> IMUSensor::getVector(adafruit_vector_type_t vector_type) {
    imu::Vector<3> xyz;
    uint8_t buffer[6] = readMultipleRegisters((adafruit_bno055_reg_t)vector_type, 6);

    int16_t x, y, z;
    x = y = z = 0;

    /* Read vector data (6 bytes) */
    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    /*!
    * Convert the value to an appropriate range (section 3.6.4)
    * and assign the value to the Vector type
    */
    switch (vector_type) {
        case VECTOR_MAGNETOMETER:
            /* 1uT = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_GYROSCOPE:
            /* 1dps = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_EULER:
            /* 1 degree = 16 LSB */
            xyz[0] = ((double)x) / 16.0;
            xyz[1] = ((double)y) / 16.0;
            xyz[2] = ((double)z) / 16.0;
            break;
        case VECTOR_ACCELEROMETER:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
        case VECTOR_LINEARACCEL:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
        case VECTOR_GRAVITY:
            /* 1m/s^2 = 100 LSB */
            xyz[0] = ((double)x) / 100.0;
            xyz[1] = ((double)y) / 100.0;
            xyz[2] = ((double)z) / 100.0;
            break;
    }

    return xyz;
}
imu::Quaternion IMUSensor::getQuat() {
    uint8_t buffer[8] = readMultipleRegisters(BNO055_QUATERNION_DATA_W_LSB_ADDR, 8);

    int16_t x, y, z, w;
    x = y = z = w = 0;

    //Bit shift data into the right places and store it
    w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
    x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
    y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
    z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

    /*!
    * Assign to Quaternion
    * See
    * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
    * 3.6.5.5 Orientation (Quaternion)
    */
    const double scale = (1.0 / (1 << 14));
    imu::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
    return quat;
}

int8_t IMUSensor::getTemp() {
    int8_t temp = (int8_t)(readSingleRegister(BNO055_TEMP_ADDR));
    return temp;
}

void IMUSensor::setAxisRemap(adafruit_bno055_axis_remap_config_t remapcode) {
    //Put into proper config for mapping stuff
    setModeTemp(OPERATION_MODE_CONFIG);
    writeRegister(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);

    //Return mode to operating mode
    setModeTemp(currentMode);
}

void IMUSensor::setAxisSign(adafruit_bno055_axis_remap_sign_t remapsign) {
    //See above method, pretty much the exact same
    setModeTemp(OPERATION_MODE_CONFIG);
    writeRegister(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
    setModeTemp(currentMode);
}    

//This method is weird; it intakes several existing byte pointers to see what action it should take. Luckily, we shouldn't have to use it.
void IMUSensor::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error) {
    //Make sure IMU is on proper register page to get system status
    writeRegister(BNO055_PAGE_ID_ADDR, 0);
    
    //If system status requested, read the status.
    if (system_status != 0) *system_status = readSingleRegister(BNO055_SYS_STAT_ADDR);
    //If self test result requested, pull the self test results.
    if (self_test_result != 0) *self_test_result = readSingleRegister(BNO055_SELFTEST_RESULT_ADDR);
    //Finally, if there's an error pull and stash it.
    if (system_error != 0) *system_error = readSingleRegister(BNO055_SYS_ERR_ADDR);
}

//Same as above method, byte pointers are fed into it as parameters that get populated by method.
void IMUSensor::getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t calData = readSingleRegister(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}

/* Functions to deal with raw calibration data */
bool IMUSensor::getSensorOffsets(uint8_t *calibData) {
    if (isFullyCalibrated()) {
        setModeTemp(OPERATION_MODE_CONFIG);

        calibData = readMultipleRegisters(ACCEL_OFFSET_X_LSB_ADDR, NUM_BNO055_OFFSET_REGISTERS);

        setModeTemp(currentMode);
        return true;
    }
    return false;
}

//Fully populated offset getter using type of offset, not just calibration data
bool IMUSensor::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type) {
    if (isFullyCalibrated()) {
        setModeTemp(OPERATION_MODE_CONFIG);

        /* Accel offset range depends on the G-range:
        +/-2g  = +/- 2000 mg
        +/-4g  = +/- 4000 mg
        +/-8g  = +/- 8000 mg
        +/-1§g = +/- 16000 mg */
        offsets_type.accel_offset_x = (readSingleRegister(ACCEL_OFFSET_X_MSB_ADDR) << 8) |
                                    (readSingleRegister(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (readSingleRegister(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |
                                    (readSingleRegister(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (readSingleRegister(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |
                                    (readSingleRegister(ACCEL_OFFSET_Z_LSB_ADDR));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type.mag_offset_x =
            (readSingleRegister(MAG_OFFSET_X_MSB_ADDR) << 8) | (readSingleRegister(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y =
            (readSingleRegister(MAG_OFFSET_Y_MSB_ADDR) << 8) | (readSingleRegister(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z =
            (readSingleRegister(MAG_OFFSET_Z_MSB_ADDR) << 8) | (readSingleRegister(MAG_OFFSET_Z_LSB_ADDR));

        /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
        500 dps = +/- 8000 LSB
        250 dps = +/- 4000 LSB
        125 dps = +/- 2000 LSB
        ... where 1 DPS = 16 LSB */
        offsets_type.gyro_offset_x =
            (readSingleRegister(GYRO_OFFSET_X_MSB_ADDR) << 8) | (readSingleRegister(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y =
            (readSingleRegister(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (readSingleRegister(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z =
            (readSingleRegister(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (readSingleRegister(GYRO_OFFSET_Z_LSB_ADDR));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type.accel_radius =
            (readSingleRegister(ACCEL_RADIUS_MSB_ADDR) << 8) | (readSingleRegister(ACCEL_RADIUS_LSB_ADDR));

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type.mag_radius =
            (readSingleRegister(MAG_RADIUS_MSB_ADDR) << 8) | (readSingleRegister(MAG_RADIUS_LSB_ADDR));

        setModeTemp(currentMode);
        return true;
    }
    return false;
}

void IMUSensor::setSensorOffsets(const uint8_t *calibData) {
    setModeTemp(OPERATION_MODE_CONFIG);

    /* Note: Configuration will take place only when user writes to the last
        byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
        Therefore the last byte must be written whenever the user wants to
        changes the configuration. */

    /* A writeLen() would make this much cleaner */
    writeRegister(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
    writeRegister(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
    writeRegister(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
    writeRegister(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
    writeRegister(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
    writeRegister(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

    writeRegister(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
    writeRegister(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
    writeRegister(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
    writeRegister(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
    writeRegister(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
    writeRegister(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

    writeRegister(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
    writeRegister(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
    writeRegister(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
    writeRegister(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
    writeRegister(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
    writeRegister(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

    writeRegister(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
    writeRegister(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

    writeRegister(MAG_RADIUS_LSB_ADDR, calibData[20]);
    writeRegister(MAG_RADIUS_MSB_ADDR, calibData[21]);

    setModeTemp(currentMode);
}

void IMUSensor::setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type) {
    setModeTemp(OPERATION_MODE_CONFIG);

    /* Note: Configuration will take place only when user writes to the last
        byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
        Therefore the last byte must be written whenever the user wants to
        changes the configuration. */

    writeRegister(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
    writeRegister(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    writeRegister(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
    writeRegister(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    writeRegister(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
    writeRegister(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    writeRegister(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
    writeRegister(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    writeRegister(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
    writeRegister(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    writeRegister(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
    writeRegister(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    writeRegister(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
    writeRegister(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    writeRegister(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
    writeRegister(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    writeRegister(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
    writeRegister(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    writeRegister(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
    writeRegister(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

    writeRegister(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
    writeRegister(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

    setModeTemp(currentMode);

}

bool IMUSensor::isFullyCalibrated() {
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);

    switch (currentMode) {
        case OPERATION_MODE_ACCONLY:
            return (accel == 3);
        case OPERATION_MODE_MAGONLY:
            return (mag == 3);
        case OPERATION_MODE_GYRONLY:
        case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
            return (gyro == 3);
        case OPERATION_MODE_ACCMAG:
        case OPERATION_MODE_COMPASS:
            return (accel == 3 && mag == 3);
        case OPERATION_MODE_ACCGYRO:
        case OPERATION_MODE_IMUPLUS:
            return (accel == 3 && gyro == 3);
        case OPERATION_MODE_MAGGYRO:
            return (mag == 3 && gyro == 3);
        default:
            return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
    }   
}

/* Power managments functions */
void IMUSensor::enterSuspendMode() {
    /* Switch to config mode (just in case since this is the default) */
    setModeTemp(OPERATION_MODE_CONFIG);
    writeRegister(BNO055_PWR_MODE_ADDR, 0x02);
    /* Set the requested operating mode (see section 3.3) */
    setModeTemp(currentMode);
}

void IMUSensor::enterNormalMode() {
    /* Switch to config mode (just in case since this is the default) */
    setModeTemp(OPERATION_MODE_CONFIG);
    writeRegister(BNO055_PWR_MODE_ADDR, 0x00);
    /* Set the requested operating mode (see section 3.3) */
    setModeTemp(modeback);
}










