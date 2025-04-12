#include "mid_imu.hpp"

//Startup routine, initialize GPIO clock output
void MidIMU::initialize() {
	//Enable 40kHz clock output on GPIO 23
	clock_gpio_init(IMU_CLOCK_GPIO, IMU_CLOCK_SOURCE_SYSTEM, IMU_CLOCK_DIVISOR);	
	sleep_ms(50);	//Allow time for clock output to stabilize
	
	//Enable both sensors and configure their ODRs
	buffer[0] = R_IMU_GYRO_CONFIG0; gyro_config0.fields.GYRO_ODR = B_IMU_GYRO_CONFIG0_ODR_500HZ;
		gyro_config0.fields.GYRO_UI_FS_SEL = B_IMU_GYRO_CONFIG0_FSR_4000DPS;
		buffer[1] = gyro_config0.data;
		write_buffer(buffer, 2, false);
	buffer[0] = R_IMU_ACCEL_CONFIG0;
		accel_config0.fields.ACCEL_ODR = B_IMU_ACCEL_CONFIG0_ODR_500HZ;
		accel_config0.fields.ACCEL_UI_FS_SEL = B_IMU_ACCEL_CONFIG0_FSR_32G;
		buffer[1] = accel_config0.data;
		write_buffer(buffer, 2, false);
	buffer[0] = R_IMU_PWR_MGMT0;
		pwr_mgmt0.fields.ACCEL_MODE = B_IMU_PWR_MGMT0_ACCEL_MODE_LOW_NOISE;
		pwr_mgmt0.fields.GYRO_MODE = B_IMU_PWR_MGMT0_GYRO_MODE_LOW_NOISE;
		pwr_mgmt0.fields.TEMP_DIS = b_IMU_PWR_MGMT0_TEMP_ENABLE;
		buffer[1] = pwr_mgmt0.data;
		write_buffer(buffer, 2, false);
	sleep_ms(1); //Datasheet instructs 200us wait after changing sensor power modes
	
	//TODO: Figure out if disabling AUX1 Serial interface is necessary 
}

//Read whoami register and return whether result matches expected response
bool MidIMU::validate() {
	//Check WHO_AM_I register and validate response
	read_buffer(R_IMU_WHO_AM_I, buffer, 1);
	return (buffer[0] == B_IMU_WHO_AM_I_VALUE);	
}

//Read all 12 data registers at once and format them into their internal fields
void MidIMU::update() {
	//Read ACCEL_DATA_X1_UI - GYRO_DATA_Z0_UI as a block
	read_buffer(R_IMU_ACCEL_DATA_X1_UI, buffer, 12);

	//Split buffer into individial fields
	int16_t int_ax, int_ay, int_az, int_gx, int_gy, int_gz = 0;
	int_ax = ((int16_t)buffer[1]) | (((int16_t)buffer[0]) << 8);
	int_ay = ((int16_t)buffer[3]) | (((int16_t)buffer[2]) << 8);
	int_az = ((int16_t)buffer[5]) | (((int16_t)buffer[4]) << 8);
	int_gx = ((int16_t)buffer[7]) | (((int16_t)buffer[6]) << 8);
	int_gy = ((int16_t)buffer[9]) | (((int16_t)buffer[8]) << 8);
	int_gz = ((int16_t)buffer[11]) | (((int16_t)buffer[10]) << 8);

	//Divide by scale factor to format as float for output
	accel_data[0] = ((float)int_ax) / S_IMU_ACCEL_SENSITIVITY_FACTOR;
	accel_data[1] = ((float)int_ay) / S_IMU_ACCEL_SENSITIVITY_FACTOR;
	accel_data[2] = ((float)int_az) / S_IMU_ACCEL_SENSITIVITY_FACTOR;
	gyro_data[0] = ((float)int_gx) / S_IMU_GYRO_SENSITIVITY_FACTOR;
	gyro_data[1] = ((float)int_gy) / S_IMU_GYRO_SENSITIVITY_FACTOR;
	gyro_data[2] = ((float)int_gz) / S_IMU_GYRO_SENSITIVITY_FACTOR;
}

//Return the gyro output as an Eigen 3-vector
Vector3f MidIMU::get_gyro_data() {
	return gyro_data;
}

//Return the accel data as an Eigen 3-vector
Vector3f MidIMU::get_accel_data() {
	return accel_data;
}

//Getters for x, y, and z of gyro data
float MidIMU::get_gyro_x() { return gyro_data[0]; }
float MidIMU::get_gyro_y() { return gyro_data[1]; }
float MidIMU::get_gyro_z() { return gyro_data[2]; }

//Getters for x, y, and z of accel data
float MidIMU::get_accel_x() { return accel_data[0]; }
float MidIMU::get_accel_y() { return accel_data[1]; }
float MidIMU::get_accel_z() { return accel_data[2]; }
