#include "high_accel.hpp"
#include "hardware/i2c.h"

//Run initialization and reset routines
void HighAccel::initialize() {
	//TODO: Refactor to consolidate into multi-register buffer writes, organize by register order and clump

	//Configure data parameters
    buffer[0] = R_ACC_BW_RATE;
	buffer[1] = B_ACC_ODR_800_HZ;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    buffer[0] = R_ACC_DATA_FORMAT;
	buffer[0] = B_ACC_DATA_FORMAT_DEFAULT;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

	//Configure interrupt settings and mapping
	buffer[0] = R_ACC_THRESH_SHOCK;
    buffer[1] = B_ACC_THRESH_SHOCK_1560mG;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    buffer[0] = R_ACC_DUR;
	buffer[1] = B_ACC_DUR_2500uS;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
	buffer[0] = R_ACC_SHOCK_AXES;
    buffer[1] = b_ACC_SHOCK_AXES_Z;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    buffer[0] = R_ACC_INT_MAP;
	buffer[1] = ~b_ACC_INT_MAP_SINGLE_SHOCK;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
	
	//Enable interrupts
    buffer[0] = R_ACC_INT_ENABLE;
	buffer[1] = b_ACC_INT_ENABLE_SINGLE_SHOCK;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

	//Set to measurement mode
    buffer[0] = R_ACC_POWER_CTL;
	buffer[1] = b_ACC_POWER_CTL_MEASURE;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

//Read devid register and confirm its validity
bool HighAccel::validate() {
	//Check the DEVID register and confirm its response
	read_register(R_ACC_DEVID, buffer, 1);
	return (buffer[0] == B_ACC_DEVID_VALUE);
}

//Read all six data registers in one operation, format them, and return as an eigen 3-vector
Eigen::Vector3f HighAccel::getData() {
	//TODO: Refactor to run as an internal member updater rather than vector return
	
	//Read DATAX0 - DATAZ1 as a block
	read_register(R_ACC_DATAX0, buffer, 6);	//TODO: Confirm this formatting works
	
	//Split buffer into individual fields
	ax = ((int16_t) buffer[0]) | ((int16_t) buffer[1] << 8);
	ay = ((int16_t) buffer[2]) | ((int16_t) buffer[3] << 8);
	az = ((int16_t) buffer[4]) | ((int16_t) buffer[5] << 8);
	
	//Divide by scale factor to format as float for output
	Eigen::Vector3f output;
	output[0] = ((float) ax) / S_ACC_SENSITIVITY_FACTOR;
	output[1] = ((float) ay) / S_ACC_SENSITIVITY_FACTOR;
	output[2] = ((float) az) / S_ACC_SENSITIVITY_FACTOR;

	return output;
}

/*
//Read interrupt source register to clear interrupt
void high_accel::clearInterrupt() {
	uint8_t intSource = read_register_buffer(R_ACC_INT_SOURCE);
}
*/
