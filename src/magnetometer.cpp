#include "magnetometer.hpp"

//Default constructor, pass I2C instance and configure address
magnetometer::magnetometer(i2c_inst_t* inst) {
	this->configureI2C(inst, MAG_I2C_ADDR);
}

//Startup routine, initialize necessary values and reset
void magnetometer::initialize() {
	//Configure decimation filter bandwidth
	write_register_byte(R_MAG_INTERNAL_CONTROL_1, B_MAG_INTERNAL_CONTROL_1_FLT_BW_800HZ);

	//Configure and enable continuous measurement mode
	write_register_byte(R_MAG_INTERNAL_CONTROL_2, (B_MAG_CM_FREQ_1000HZ | 
												   b_MAG_CMM_EN));

	//Initial calibration of bridge offset on startup
	calibrateBridgeOffset();
}

//Read product ID register and return whether result matches expected (default) response
bool magnetometer::validate() {
	return check_register_byte(R_MAG_PRODUCT_ID, B_MAG_PRODUCT_ID_VALUE);	
}

//Read all 7 data registers, split them, and scale them properly to return as Eigen 3-vector
Vector3f magnetometer::getData() {
	//Read Xout1 - XYZout2 as a block
	uint8_t dataBuffer[7] = read_register_buffer(R_MAG_XOUT0, 7);

	//Split into fields
	int32_t x, y, z = 0;
	x = ((int32_t)dataBuffer[0] << 10) | ((int32_t)dataBuffer[1] << 2) | ((int32_t)(dataBuffer[6] & 0xC0) >> 6);
	y = ((int32_t)dataBuffer[2] << 10) | ((int32_t)dataBuffer[3] << 2) | ((int32_t)(dataBuffer[6] & 0x30) >> 4);
	z = ((int32_t)dataBuffer[4] << 10) | ((int32_t)dataBuffer[5] << 2) | ((int32_t)(dataBuffer[6] & 0x0C) >> 2);

	//Divide by scale factor to format as float for output
	Vector3f output;
	output[0] = ((float)x) / S_MAG_SENSITIVITY_FACTOR;
	output[1] = ((float)y) / S_MAG_SENSITIVITY_FACTOR;
	output[2] = ((float)z) / S_MAG_SENSITIVITY_FACTOR;
	
	//Subtract bridge offset from output and return
	output -= bridgeOffset;
	return output;
}

//Perform set/reset to calibrate for temperature changes
void calibrateBridgeOffset() {
	//Perform set operation and take measurement
	write_register_byte(R_MAG_INTERNAL_CONTROL_0, b_MAG_INTERNAL_CONTROL_0_SET);
	//TODO: Confirm 500ns delay requested by datasheet isn't explicitly necessary in function call
	Vector3f setOutput = getData();

	//Perform reset operation and take measurement
	write_register_byte(R_MAG_INTERNAL_CONTROL_0, b_MAG_INTERNAL_CONTROL_0_RESET);
	Vector3f resetOutput = getData;

	//Calculate offset and store as per datasheet instructions, and perform second set operation
	bridgeOffset = (setOutput + resetOutput) / 2;

	//Perform second set operation to return sensor to normal polarity
	write_register_byte(R_MAG_INTERNAL_CONTROL_0, b_MAG_INTERNAL_CONTROL_0_SET);
}
