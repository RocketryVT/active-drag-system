#include "low_imu.hpp"

//Default constructor, pass I2C instance
low_imu::low_imu(i2c_inst_t* inst) {
	this->inst = inst;
}

//Do initialization and reset routines
void low_imu::initialize() {
	//TODO: Figure out startup configuration necessary
}

//Read whoami register and confirm whether response matches expected
bool low_imu::validate() {
	uint8_t command;

	//Request whoami data from sensor register
	command = c_READ_WHOAMI;
	i2c_write_blocking(this->inst, LOW_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, LOW_I2C_ADDR, buffer, 1, false);
	
	//Check the whoami value and confirm it matches the datasheet
	if (buffer[0] == v_WHOAMI_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
