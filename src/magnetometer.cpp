#include "magnetometer.hpp"

//Default constructor, pass I2C instance
magnetometer::magnetometer(i2c_inst_t* inst) {
	this->inst = inst;
}

//Startup routine, initialize necessary values and reset
void magnetometer::initialize() {
	//TODO: Figure out what initialization is necessary for magnetometer
}

//Read product ID register and return whether result matches expected (default) response
bool magnetometer::validate() {
	uint8_t command;

	//Send request for register and read it out
	command = c_READ_PRODID;
	i2c_write_blocking(this->inst, MAG_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, MAG_I2C_ADDR, this->buffer, 1, false);

	//Confirm whether response matches datasheet
	if (buffer[0] == v_PRODID_RESPONSE) {
		return true;
	} else {
		return false;
	}
}
