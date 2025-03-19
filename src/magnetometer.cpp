#include "magnetometer.hpp"

//Default constructor, pass I2C instance and configure address
Magnetometer::Magnetometer(i2c_inst_t* inst) {
	this->bus = inst;
	this->addr = MAG_I2C_ADDR;
}

//Startup routine, initialize necessary values and reset
void Magnetometer::initialize() {
	//Configure decimation filter bandwidth
	buffer[0] = B_MAG_INTERNAL_CONTROL_1_FLT_BW_800HZ;
	write_buffer(R_MAG_INTERNAL_CONTROL_1, buffer, 1);

	//Configure and enable continuous measurement mode
	buffer[0] = (B_MAG_INTERNAL_CONTROL_2_CM_FREQ_1000HZ | 
				 b_MAG_INTERNAL_CONTROL_2_CMM_EN);
	write_buffer(R_MAG_INTERNAL_CONTROL_2, buffer, 1);
	
	//TODO: Figure out what amount of calibration the bridge offset actually adds/if it's necessary
	//Initial calibration of bridge offset on startup
	//calibrateBridgeOffset();
}

//Read product ID register and return whether result matches expected (default) response
bool Magnetometer::validate() {
	read_buffer(R_MAG_PRODUCT_ID, buffer, 1);
	return (buffer[0] == B_MAG_PRODUCT_ID_VALUE);	
}

//Read all 7 data registers, split them, and scale them properly to return as Eigen 3-vector
Vector3f Magnetometer::getData() {
	//TODO: Refactor as update function that dumps to internal fields
	
	//Read Xout1 - XYZout2 as a block
   	read_buffer(R_MAG_XOUT0, buffer, 7);

	//Split into fields
	int32_t x, y, z = 0;
	x = ((int32_t)buffer[0] << 10) | ((int32_t)buffer[1] << 2) | ((int32_t)(buffer[6] & 0xC0) >> 6);
	y = ((int32_t)buffer[2] << 10) | ((int32_t)buffer[3] << 2) | ((int32_t)(buffer[6] & 0x30) >> 4);
	z = ((int32_t)buffer[4] << 10) | ((int32_t)buffer[5] << 2) | ((int32_t)(buffer[6] & 0x0C) >> 2);

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
void Magnetometer::calibrateBridgeOffset() {
	//Perform set operation and take measurementi
	buffer[0] = b_MAG_INTERNAL_CONTROL_0_SET;
	write_buffer(R_MAG_INTERNAL_CONTROL_0, buffer, 1);
	//TODO: Confirm 500ns delay requested by datasheet isn't explicitly necessary in function call
	Vector3f setOutput = getData();

	//Perform reset operation and take measurement
	buffer[0] = b_MAG_INTERNAL_CONTROL_0_RESET;
	write_buffer(R_MAG_INTERNAL_CONTROL_0, buffer, 1);
	Vector3f resetOutput = getData();

	//Calculate offset and store as per datasheet instructions, and perform second set operation
	bridgeOffset = (setOutput + resetOutput) / 2;

	//Perform second set operation to return sensor to normal polarity
	buffer[0] = b_MAG_INTERNAL_CONTROL_0_SET;
	write_buffer(R_MAG_INTERNAL_CONTROL_0, buffer, 1);
}
