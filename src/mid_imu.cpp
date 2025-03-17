#include "mid_imu.hpp"

//Default constructor, pass I2C instance and address
mid_imu::mid_imu(i2c_inst_t* inst) {
	this->configureI2C(inst, IMU_I2C_ADDR);
}

//Startup routine, initialize GPIO clock output
void mid_imu::initialize() {
	//Enable 40kHz clock output on GPIO 21
	//0x06 is for clock source, 3125 is 125MHz to 40kHz divider
	clock_gpio_init(21, 0x6, 3125);	//TODO: Store these constants somewhere?
	sleep_ms(10);	//Allow time for clock to stabilize
	
	//Enable both sensors and configure their ODRs
	write_register_byte(R_IMU_GYRO_CONFIG0, B_IMU_GYRO_CONFIG0_ODR_500HZ);
	write_register_byte(R_IMU_ACCEL_CONFIG0, B_IMU_ACCEL_CONFIG0_ODR_500HZ);
	write_register_byte(R_IMU_PWR_MGMT0, (B_IMU_PWR_MGMT0_GYRO_MODE_LN | 
										  B_IMU_PWR_MGMT0_ACCEL_MODE_LN));
	sleep_ms(1); //Datasheet instructs 200us wait after changing sensor power modes
	
	//TODO: Figure out if disabling AUX1 Serial interface is necessary 
}

//Read whoami register and return whether result matches expected response
bool mid_imu::validate() {
	//Check WHO_AM_I register and validate response
	return check_register_byte(R_IMU_WHO_AM_I, B_IMU_WHO_AM_I_VALUE);	
}

//Read all 12 data registers at once, format them, and return as an Eigen 6-vector (Accel, Gyro)
Vector6f mid_imu::getData() {
	//Read ACCEL_DATA_X1_UI - GYRO_DATA_Z0_UI as a block
	uint8_t dataBuffer[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	read_register_buffer_into_array(R_IMU_ACCEL_DATA_X1_UI, dataBuffer, 12);	//TODO: Confirm this formatting works

	//Split buffer into individial fields
	int16_t ax, ay, az, gx, gy, gz = 0;
	ax = ((int16_t)dataBuffer[1]) | (((int16_t)dataBuffer[0]) << 8);
	ay = ((int16_t)dataBuffer[3]) | (((int16_t)dataBuffer[2]) << 8);
	az = ((int16_t)dataBuffer[5]) | (((int16_t)dataBuffer[4]) << 8);
	gx = ((int16_t)dataBuffer[7]) | (((int16_t)dataBuffer[6]) << 8);
	gy = ((int16_t)dataBuffer[9]) | (((int16_t)dataBuffer[8]) << 8);
	gz = ((int16_t)dataBuffer[11]) | (((int16_t)dataBuffer[10]) << 8);

	//Divide by scale factor to format as float for output
	Vector6f output;
	output[0] = ((float)ax) / S_IMU_ACCEL_SENSITIVITY_FACTOR;
	output[1] = ((float)ay) / S_IMU_ACCEL_SENSITIVITY_FACTOR;
	output[2] = ((float)az) / S_IMU_ACCEL_SENSITIVITY_FACTOR;
	output[3] = ((float)gx) / S_IMU_GYRO_SENSITIVITY_FACTOR;
	output[4] = ((float)gy) / S_IMU_GYRO_SENSITIVITY_FACTOR;
	output[5] = ((float)gz) / S_IMU_GYRO_SENSITIVITY_FACTOR;

	return output;
}
