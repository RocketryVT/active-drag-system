#include "adxl375.hpp"

void ADXL375::initialize() {
    //Configure power mode and ODR
    buffer[0] = R_ADXL375_BW_RATE;
    bw_rate.fields.LOW_POWER = B_ADXL375_BW_RATE_NORMAL_POWER_MODE;
    bw_rate.fields.RATE = B_ADXL375_ODR_800_HZ;
    buffer[1] = bw_rate.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Configure data output format
    buffer[0] = R_ADXL375_DATA_FORMAT;
    i2c_write_blocking(i2c, addr, buffer, 1, false);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    //Some reserved fields are set to 1, read byte and *then* set desired bits
    data_format.data = buffer[0];

    buffer[0] = R_ADXL375_DATA_FORMAT;
    data_format.fields.JUSTIFY = B_ADXL375_DATA_FORMAT_JUSTIFY_RIGHT;
    data_format.fields.INT_INVERT = B_ADXL375_DATA_FORMAT_INTERRUPT_ACTIVE_HIGH;
    buffer[1] = data_format.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Set to measurement mode
    buffer[0] = R_ADXL375_POWER_CTL;
    power_ctl.fields.MEASURE = true;
    buffer[1] = power_ctl.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

void ADXL375::sample() {
    //Read DATAX0 - DATAZ1 as a block
    buffer[0] = R_ADXL375_DATAX0;
    i2c_write_blocking(i2c, addr, buffer, 1, false);
    i2c_read_blocking(i2c, addr, buffer, 6, false);

    //Split buffer into individual fields
    ax = ((int16_t) buffer[0]) | ((int16_t) buffer[1] << 8);
    ay = ((int16_t) buffer[2]) | ((int16_t) buffer[3] << 8);
    az = ((int16_t) buffer[4]) | ((int16_t) buffer[5] << 8);
}

