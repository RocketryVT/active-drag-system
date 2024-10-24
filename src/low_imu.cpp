#include "low_imu.hpp"

low_imu::low_imu(i2c_inst_t* inst) {
	this->inst = inst;
}

void low_imu::initialize() {
	//TODO: Figure out startup configuration necessary
}

bool low_imu::validate() {
	// Read whoami and return its validity
	this->buffer[0] = c_READ_WHOAMI;
	i2c_read_blocking(this->inst, I2C_ADDR, this->buffer, 1, true);
	
	if (buffer[0] == v_WHOAMI_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
