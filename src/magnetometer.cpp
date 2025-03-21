#include "magnetometer.hpp"

//Startup routine, initialize necessary values and reset
void Magnetometer::initialize() {
	//Configure decimation filter bandwidth
	buffer[0] = B_MAG_INTERNAL_CONTROL_1_FLT_BW_800HZ;
	write_register(R_MAG_INTERNAL_CONTROL_1, buffer, 1);

	//Configure and enable continuous measurement mode
	buffer[0] = (B_MAG_INTERNAL_CONTROL_2_CM_FREQ_1000HZ | 
				 b_MAG_INTERNAL_CONTROL_2_CMM_EN);
	write_register(R_MAG_INTERNAL_CONTROL_2, buffer, 1);
	
	//TODO: Figure out what amount of calibration the bridge offset actually adds/if it's necessary
	//Initial calibration of bridge offset on startup
	//calibrateBridgeOffset();
}

//Read product ID register and return whether result matches expected (default) response
bool Magnetometer::validate() {
	read_register(R_MAG_PRODUCT_ID, buffer, 1);
	return (buffer[0] == B_MAG_PRODUCT_ID_VALUE);	
}

//Read all 7 data registers, split them, and scale them properly to return as Eigen 3-vector
Eigen::Vector3f Magnetometer::getData() {
	//TODO: Refactor as update function that dumps to internal fields
	
	//Read Xout1 - XYZout2 as a block
   	read_register(R_MAG_XOUT0, buffer, 7);

	//Split into fields
	ax = ((int32_t)buffer[0] << 10) | ((int32_t)buffer[1] << 2) | ((int32_t)(buffer[6] & 0xC0) >> 6);
	ay = ((int32_t)buffer[2] << 10) | ((int32_t)buffer[3] << 2) | ((int32_t)(buffer[6] & 0x30) >> 4);
	az = ((int32_t)buffer[4] << 10) | ((int32_t)buffer[5] << 2) | ((int32_t)(buffer[6] & 0x0C) >> 2);

	//Divide by scale factor to format as float for output
	Eigen::Vector3f output;
	output[0] = ((float)ax) / S_MAG_SENSITIVITY_FACTOR;
	output[1] = ((float)ay) / S_MAG_SENSITIVITY_FACTOR;
	output[2] = ((float)az) / S_MAG_SENSITIVITY_FACTOR;
	
	//Subtract bridge offset from output and return
	output -= bridgeOffset;
	return output;
}
