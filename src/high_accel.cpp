#include "high_accel.hpp"

//Default constructor, pass I2C instance
high_accel::high_accel(i2c_inst_t* inst) {
	//Configure I2C instance
	this->bus = inst;
	this->bus_addr = HIGH_I2C_ADDR;
}

//Run initialization and reset routines
void high_accel::initialize() {
	//TODO: Refactor to use multi-register buffer writes to simplify process

	//Configure data parameters
	write_register_byte(R_ACC_BW_RATE, B_ACC_ODR_800_HZ);
	write_register_byte(R_ACC_DATA_FORMAT, B_ACC_DATA_FORMAT_DEFAULT);

	//Configure interrupt settings and mapping
	write_register_byte(R_ACC_THRESH_SHOCK, B_ACC_THRESH_SHOCK_1560mG);
	write_register_byte(R_ACC_DUR, B_ACC_DUR_2500uS);
	write_register_byte(R_ACC_SHOCK_AXES, b_ACC_SHOCK_AXES_Z);		//TODO: Figure out way to do axis handling
	write_register_byte(R_ACC_INT_MAP, ~b_ACC_INT_MAP_SINGLE_SHOCK);	//TODO: Confirm mapping (0 = INT1, 1 = INT2)
	
	//Enable interrupts
	write_register_byte(R_ACC_INT_ENABLE, b_ACC_INT_ENABLE_SINGLE_SHOCK);

	//Set to measurement mode
	write_register_byte(R_ACC_POWER_CTL, b_ACC_POWER_CTL_MEASURE);
}

//Read devid register and confirm its validity
bool high_accel::validate() {
	//Check the DEVID register and confirm its response
	return this->check_register_byte(R_ACC_DEVID, B_ACC_DEVID_VALUE);
}

//Read all six data registers in one operation, format them, and return as an eigen 3-vector
Vector3f high_accel::getData() {
	//Read DATAX0 - DATAZ1 as a block
	uint8_t dataBuffer[6] = {0, 0, 0, 0, 0, 0};
	read_register_buffer(R_ACC_DATAX0, dataBuffer, 6);	//TODO: Confirm this formatting works
	
	//Split buffer into individual fields
	int16_t x, y, z = 0;
	x = ((int16_t) dataBuffer[0]) | ((int16_t) dataBuffer[1] << 8);
	y = ((int16_t) dataBuffer[2]) | ((int16_t) dataBuffer[3] << 8);
	z = ((int16_t) dataBuffer[4]) | ((int16_t) dataBuffer[5] << 8);
	
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
