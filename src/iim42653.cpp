#include "iim42653.hpp"

//Startup routine, initialize GPIO clock output
void IIM42653::initialize() {
    //Enable 40kHz clock output on GPIO 23
    clock_gpio_init(MICRO_DEFAULT_CLK_OUTPUT, IIM42653_CLOCK_SOURCE_SYSTEM, IIM42653_CLOCK_DIVISOR);	
    sleep_ms(50);	//Allow time for clock output to stabilize

    //Enable both sensors and configure their ODRs
    buffer[0] = R_IIM42653_GYRO_CONFIG0;
    gyro_config0.fields.GYRO_ODR = B_IIM42653_GYRO_CONFIG0_ODR_500HZ;
    gyro_config0.fields.GYRO_UI_FS_SEL = B_IIM42653_GYRO_CONFIG0_FSR_4000DPS;
    buffer[1] = gyro_config0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    buffer[0] = R_IIM42653_ACCEL_CONFIG0;
    accel_config0.fields.ACCEL_ODR = B_IIM42653_ACCEL_CONFIG0_ODR_500HZ;
    accel_config0.fields.ACCEL_UI_FS_SEL = B_IIM42653_ACCEL_CONFIG0_FSR_32G;
    buffer[1] = accel_config0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    buffer[0] = R_IIM42653_PWR_MGMT0;
    pwr_mgmt0.fields.ACCEL_MODE = B_IIM42653_PWR_MGMT0_ACCEL_MODE_LOW_NOISE;
    pwr_mgmt0.fields.GYRO_MODE = B_IIM42653_PWR_MGMT0_GYRO_MODE_LOW_NOISE;
    pwr_mgmt0.fields.TEMP_DIS = B_IIM42653_PWR_MGMT0_TEMP_ENABLE;
    buffer[1] = pwr_mgmt0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    sleep_ms(1); //Datasheet instructs 200us wait after changing sensor power modes

    //TODO: Figure out if disabling AUX1 Serial interface is necessary 
}

//Read all 12 data registers at once and format them into their internal fields
void IIM42653::sample() {
    //Read ACCEL_DATA_X1_UI - GYRO_DATA_Z0_UI as a block
    buffer[0] = R_IIM42653_ACCEL_DATA_X1_UI;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 12, false);

    //Split buffer into individial fields
    ax = ((int16_t)buffer[1]) | (((int16_t)buffer[0]) << 8);
    ay = ((int16_t)buffer[3]) | (((int16_t)buffer[2]) << 8);
    az = ((int16_t)buffer[5]) | (((int16_t)buffer[4]) << 8);
    gx = ((int16_t)buffer[7]) | (((int16_t)buffer[6]) << 8);
    gy = ((int16_t)buffer[9]) | (((int16_t)buffer[8]) << 8);
    gz = ((int16_t)buffer[11]) | (((int16_t)buffer[10]) << 8);
}

