#include "mid_imu.hpp"

//Default constructor, pass I2C instance
mid_imu::mid_imu(i2c_inst_t* inst) {
	this->inst = inst;
}

//Startup routine, initialize GPIO clock output
void mid_imu::initialize() {
	//Enable 40kHz clock output on GPIO 21
	//0x06 is for clock source, 3125 is 125MHz to 40kHz divider
	clock_gpio_init(21, 0x6, 3125);	//TODO: Store these constants somewhere?

	//TODO: Figure out startup configuration necessary, especially for disabling AUX1 Serial interface
}

//Read whoami register and return whether result matches expected response
bool mid_imu::validate() {
	uint8_t command;
	
	//Send request for register and read it out
	command = c_READ_WHOAMI;
	i2c_write_blocking(this->inst, MID_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, MID_I2C_ADDR, this->buffer, 1, false);
	
	//Confirm whether response matches datasheet
	if (buffer[0] == v_WHOAMI_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
