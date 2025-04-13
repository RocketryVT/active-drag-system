#include "mmc5983ma.hpp"

int16_t MMC5983MA::sat_sub(int16_t a, int16_t b) {
    int32_t result = (int32_t) a - (int32_t) b;
    if (result < INT16_MIN) {
        result = INT16_MIN;
    }
    if (result > INT16_MAX) {
        result = INT16_MAX;
    }
    return (int16_t) result;
};

void MMC5983MA::initialize() {
    // Restart device prior to configuration
    buffer[0] = R_MMC5983MA_INTERNAL_CTL1;
    internal_ctl1.fields.RESTART = true;
    buffer[1] = internal_ctl1.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    internal_ctl1.data = 0;

    sleep_ms(100);

    calibrate();

    //Configure decimation filter bandwidth for 200 Hz
    buffer[0] = R_MMC5983MA_INTERNAL_CTL1;
    internal_ctl1.fields.BANDWIDTH = B_MMC5983MA_BANDWIDTH_200HZ;
    buffer[1] = internal_ctl1.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Configure and enable continuous measurement mode for 200 Hz
    buffer[0] = R_MMC5983MA_INTERNAL_CTL2;
    internal_ctl2.fields.CONTINUOUS_MODE_ENABLE = true;
    internal_ctl2.fields.CONTINUOUS_MODE_FREQ = B_MMC5983_CONTINUOUS_MODE_FREQ_200HZ;
    internal_ctl2.fields.PERIODIC_SET_ENABLE = false;
    internal_ctl2.fields.PERIODIC_SET_RATE = B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_1000;
    buffer[1] = internal_ctl2.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

void MMC5983MA::sample() {
    buffer[0] = R_MMC5983MA_XOUT0;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 6, false);

    raw_x = (int16_t) ((((uint16_t) buffer[0] << 8) | buffer[1]) - 32768);
    raw_y = (int16_t) ((((uint16_t) buffer[2] << 8) | buffer[3]) - 32768);
    raw_z = (int16_t) ((((uint16_t) buffer[4] << 8) | buffer[5]) - 32768);
}

void MMC5983MA::apply_offset() {
    ax = sat_sub(raw_x, offset_x);
    ay = sat_sub(raw_y, offset_y);
    az = sat_sub(raw_z, offset_z);
}

void MMC5983MA::calibrate() {
    sleep_ms(100);

    // Take measurement after SET command completes
    buffer[0] = R_MMC5983MA_INTERNAL_CTL0;
    internal_ctl0.fields.SET_CMD = true;
    buffer[1] = internal_ctl0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    internal_ctl0.data = 0;

    sleep_ms(100);

    buffer[0] = R_MMC5983MA_INTERNAL_CTL0;
    internal_ctl0.fields.TAKE_MAG_MEAS = true;
    buffer[1] = internal_ctl0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    internal_ctl0.data = 0;

    while (!dev_status.fields.MEAS_M_DONE) {
        sleep_ms(1);
        buffer[0] = R_MMC5983MA_DEV_STATUS;
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        dev_status.data = buffer[0];
    }

    sample();

    int16_t set_ax = raw_x, set_ay = raw_y, set_az = raw_z;

    // Take measurement after RESET command completes
    buffer[0] = R_MMC5983MA_INTERNAL_CTL0;
    internal_ctl0.fields.RESET_CMD = true;
    buffer[1] = internal_ctl0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    internal_ctl0.data = 0;

    sleep_ms(100);

    buffer[0] = R_MMC5983MA_INTERNAL_CTL0;
    internal_ctl0.fields.TAKE_MAG_MEAS = true;
    buffer[1] = internal_ctl0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    internal_ctl0.data = 0;

    while (!dev_status.fields.MEAS_M_DONE) {
        sleep_ms(1);
        buffer[0] = R_MMC5983MA_DEV_STATUS;
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        dev_status.data = buffer[0];
    }

    sample();

    int16_t reset_ax = raw_x, reset_ay = raw_y, reset_az = raw_z;

    sleep_ms(100);

    // Average the two measurements taken and assign them as offsets
    offset_x = (int16_t) (((int32_t) set_ax + (int32_t) reset_ax) / 2);
    offset_y = (int16_t) (((int32_t) set_ay + (int32_t) reset_ay) / 2);
    offset_z = (int16_t) (((int32_t) set_az + (int32_t) reset_az) / 2);

#if ( DEBUG == 1 )
    printf("MMC5983MA: Calibrated!\n\toffset_x: %" PRIi16 "\n\toffset_y: %" PRIi16 "\n\toffset_z: %" PRIi16 "\n", offset_x, offset_y, offset_z);
#endif
}
