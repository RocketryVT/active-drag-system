#include "mid.hpp"

mid_imu::mid_imu(i2c_inst_t* inst) {
	this->inst = inst;
}

void mid_imu::initialize() {
	//Enable 40kHz clock output on GPIO 21
	//0x06 is for clock source, 3125 is 125MHz to 40kHz divider
	clock_gpio_init(21, 0x6, 3125);	

	//TODO: Figure out startup configuration necessary, especially for disabling AUX1 Serial interface

}

bool mid_imu::validate() {
	// Read whoami and return its validity
	this->buffer[0] = c_READ_WHOAMI;
	i2c_read_blocking(this->inst, I2C_ADDR, this->buffer, 1, true);
	
	if (buffer[0] == v_WHOAMI_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
