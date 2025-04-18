#include "iim42653.hpp"

int16_t IIM42653::sat_sub(int16_t a, int16_t b) {
    int32_t result = (int32_t) a - (int32_t) b;
    if (result < INT16_MIN) {
        result = INT16_MIN;
    }
    if (result > INT16_MAX) {
        result = INT16_MAX;
    }
    return (int16_t) result;
};

//Startup routine, initialize GPIO clock output
void IIM42653::initialize() {
    //Enable 40kHz clock output on GPIO 23
    clock_gpio_init(MICRO_DEFAULT_CLK_OUTPUT, IIM42653_CLOCK_SOURCE_SYSTEM, IIM42653_CLOCK_DIVISOR);	
    sleep_ms(50);	//Allow time for clock output to stabilize

    //Configure Gyroscope FSR and ODR
    buffer[0] = R_IIM42653_GYRO_CONFIG0;
    gyro_config0.fields.GYRO_ODR = B_IIM42653_GYRO_CONFIG0_ODR_500HZ;
    gyro_config0.fields.GYRO_UI_FS_SEL = B_IIM42653_GYRO_CONFIG0_FSR_4000DPS;
    buffer[1] = gyro_config0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    
    //Configure Accelerometer FSR and ODR
    buffer[0] = R_IIM42653_ACCEL_CONFIG0;
    accel_config0.fields.ACCEL_ODR = B_IIM42653_ACCEL_CONFIG0_ODR_500HZ;
    accel_config0.fields.ACCEL_UI_FS_SEL = B_IIM42653_ACCEL_CONFIG0_FSR_32G;
    buffer[1] = accel_config0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    
    //Enable Gyroscope, Accelerometer, and Thermocouple
    buffer[0] = R_IIM42653_PWR_MGMT0;
    pwr_mgmt0.fields.ACCEL_MODE = B_IIM42653_PWR_MGMT0_ACCEL_MODE_LOW_NOISE;
    pwr_mgmt0.fields.GYRO_MODE = B_IIM42653_PWR_MGMT0_GYRO_MODE_LOW_NOISE;
    pwr_mgmt0.fields.TEMP_DIS = B_IIM42653_PWR_MGMT0_TEMP_ENABLE;
    buffer[1] = pwr_mgmt0.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    sleep_ms(10); //Datasheet instructs 200us wait after changing sensor power modes
    
    //Calibrate gyro bias once all sensors are initialized
    calibrate_gyro();
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
    raw_gx = ((int16_t)buffer[7]) | (((int16_t)buffer[6]) << 8);
    raw_gy = ((int16_t)buffer[9]) | (((int16_t)buffer[8]) << 8);
    raw_gz = ((int16_t)buffer[11]) | (((int16_t)buffer[10]) << 8);
}
void IIM42653::apply_gyro_offset() {
    gx = sat_sub(raw_gx, bias_gx);
    gy = sat_sub(raw_gy, bias_gy);
    gz = sat_sub(raw_gz, bias_gz);
}

//Request certain number of gyroscope readings, and set the gyro bias values to the averages of the readings
void IIM42653::calibrate_gyro() {
    //Initialize buffers
    int64_t g_x = 0;
    int64_t g_y = 0;
    int64_t g_z = 0;

    //Request configured number of samples and l
    for (int64_t i = 0; i < n_gyro_bias_readings; i++) {
        sample();
        g_x += raw_gx;
        g_y += raw_gy;
        g_z += raw_gz;
        sleep_ms(5);
    }

    //Set bias values to the average readings during the calibration period
    bias_gx = g_x/n_gyro_bias_readings;
    bias_gy = g_y/n_gyro_bias_readings;
    bias_gz = g_z/n_gyro_bias_readings;
}


