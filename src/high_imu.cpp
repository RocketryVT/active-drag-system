#include "high_imu.hpp"

//Default constructor, pass I2C instance
high_imu::high_imu(i2c_inst_t* inst) {
	this->inst = inst;
}

//Run initialization and reset routines
void high_imu::initialize() {
	//TODO: Figure out startup configuration necessary
}

//Read devid register and confirm its validity
bool high_imu::validate() {
	uint8_t command;
   	
	//Request information from sensor register
	command	= c_READ_DEVID;
	i2c_write_blocking(this->inst, HIGH_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, HIGH_I2C_ADDR, buffer, 1, false);

	//Check validity of devid register
	if (buffer[0] == v_DEVID_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
