#include "high_accel.hpp"

//Default constructor, pass I2C instance
HighAccel::HighAccel(i2c_inst_t* inst) {
	//Configure I2C instance
	this->bus = inst;
	this->bus_addr = HIGH_I2C_ADDR;
}

//Run initialization and reset routines
void HighAccel::initialize() {
	//TODO: Refactor to consolidate into multi-register buffer writes, organize by register order and clump

	//Configure data parameters
	buffer[0] = B_ACC_ODR_800_HZ;
	write_buffer(R_ACC_BW_RATE, buffer, 1);
	buffer[0] = B_ACC_ODR_800_HZ;
	write_buffer(R_ACC_BW_RATE, buffer, 1);
	buffer[0] = B_ACC_DATA_FORMAT_DEFAULT;
	write_buffer(R_ACC_DATA_FORMAT, buffer, 1);

	//Configure interrupt settings and mapping
	buffer[0] = B_ACC_THRESH_SHOCK_1560mG;
	write_buffer(R_ACC_THRESH_SHOCK, buffer, 1);
	buffer[0] = B_ACC_DUR_2500uS;
	write_buffer(R_ACC_DUR, buffer, 1);
	buffer[0] = b_ACC_SHOCK_AXES_Z;
	write_buffer(R_ACC_SHOCK_AXES, buffer, 1);	//TODO: Figure out way to do axis handling
	buffer[0] = ~b_ACC_INT_MAP_SINGLE_SHOCK;
	write_buffer(R_ACC_INT_MAP, buffer, 1);		//TODO: Confirm mapping (0 = INT1, 1 = INT2)
	
	//Enable interrupts
	buffer[0] = b_ACC_INT_ENABLE_SINGLE_SHOCK;
	write_buffer(R_ACC_INT_ENABLE, buffer, 1);

	//Set to measurement mode
	buffer[0] = b_ACC_POWER_CTL_MEASURE;
	write_buffer(R_ACC_POWER_CTL, buffer, 1);
}

//Read devid register and confirm its validity
bool HighAccel::validate() {
	//Check the DEVID register and confirm its response
	read_buffer(R_ACC_DEVID, buffer, 1);
	return (buffer[0] == B_ACC_DEVID_VALUE);
}

//Read all six data registers in one operation, format them, and return as an eigen 3-vector
Vector3f HighAccel::getData() {
	//Read DATAX0 - DATAZ1 as a block
	read_register_buffer(R_ACC_DATAX0, buffer, 6);	//TODO: Confirm this formatting works
	
	//Split buffer into individual fields
	int16_t x, y, z = 0;
	x = ((int16_t) buffer[0]) | ((int16_t) buffer[1] << 8);
	y = ((int16_t) buffer[2]) | ((int16_t) buffer[3] << 8);
	z = ((int16_t) buffer[4]) | ((int16_t) buffer[5] << 8);
	
	//Divide by scale factor to format as float for output
	Vector3f output;
	output[0] = ((float) x) / S_ACC_SENSITIVITY_FACTOR;
	output[1] = ((float) y) / S_ACC_SENSITIVITY_FACTOR;
	output[2] = ((float) z) / S_ACC_SENSITIVITY_FACTOR;

	return output;
}

/*
//Read interrupt source register to clear interrupt
void high_accel::clearInterrupt() {
	uint8_t intSource = read_register_buffer(R_ACC_INT_SOURCE);
}
*/
