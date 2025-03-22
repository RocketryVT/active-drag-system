#include "mid_imu.hpp"
#include "hardware/i2c.h"
#include "magnetometer.hpp"

//Startup routine, initialize GPIO clock output
void MidIMU::initialize() {
	//Enable 40kHz clock output on GPIO 23
	//0x06 is for clock source, 3125 is 125MHz to 40kHz divider
	clock_gpio_init(23, 0x6, 3125);	//TODO: Store these constants somewhere?
	sleep_ms(100);	//Allow time for clock to stabilize
	
	//TODO: Refactor to consolidate sequential register writes into one operation

	//Enable both sensors and configure their ODRs
    buffer[0] = R_IMU_GYRO_CONFIG0;
	buffer[1] = B_IMU_GYRO_CONFIG0_ODR_500HZ;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    buffer[0] = R_IMU_ACCEL_CONFIG0;
	buffer[1] = B_IMU_ACCEL_CONFIG0_ODR_500HZ;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    buffer[0] = R_IMU_PWR_MGMT0;
	buffer[1] = (B_IMU_PWR_MGMT0_GYRO_MODE_LN | 
				 B_IMU_PWR_MGMT0_ACCEL_MODE_LN);
    i2c_write_blocking(i2c, addr, buffer, 2, false);
	sleep_ms(1); //Datasheet instructs 200us wait after changing sensor power modes
	
	//TODO: Figure out if disabling AUX1 Serial interface is necessary 
}

//Read whoami register and return whether result matches expected response
bool MidIMU::validate() {
	//Check WHO_AM_I register and validate response
	read_register(R_IMU_WHO_AM_I, buffer, 1);
	return (buffer[0] == B_IMU_WHO_AM_I_VALUE);	
}

//Read all 12 data registers at once, format them, and return as an Eigen 6-vector (Accel, Gyro)
void MidIMU::getData() {
	//Read ACCEL_DATA_X1_UI - GYRO_DATA_Z0_UI as a block
	read_register(R_IMU_ACCEL_DATA_X1_UI, buffer, 12);	//TODO: Confirm this formatting works

	//Split buffer into individial fields
	ax = ((int16_t)buffer[1]) | (((int16_t)buffer[0]) << 8);
	ay = ((int16_t)buffer[3]) | (((int16_t)buffer[2]) << 8);
	az = ((int16_t)buffer[5]) | (((int16_t)buffer[4]) << 8);
	gx = ((int16_t)buffer[7]) | (((int16_t)buffer[6]) << 8);
	gy = ((int16_t)buffer[9]) | (((int16_t)buffer[8]) << 8);
	gz = ((int16_t)buffer[11]) | (((int16_t)buffer[10]) << 8);

}
