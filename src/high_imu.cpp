#include "high_imu.hpp"

high_imu::high_imu(i2c_inst_t* inst) {
	this->inst = inst;
}

void high_imu::initialize() {
	//TODO: Figure out startup configuration necessary
}

bool high_imu::validate() {
	// Read device ID and return its validity
	this->buffer[0] = c_READ_DEVID;
	i2c_read_blocking(this->inst, I2C_ADDR, this->buffer, 1, true);
	
	if (buffer[0] == v_DEVID_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
