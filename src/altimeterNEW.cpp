#include "altimeterNEW.hpp"

//TODO: Update class definitions so this class steals the moniker of [altimeter] and old class is deprecated
altimeterNEW::altimeterNEW(i2c_inst_t* inst) {
	this->inst = inst;
}

void altimeterNEW::initialize() {
	// Send reset command to sensor
	this->buffer[0] = c_RESET;
	i2c_write_blocking(this->inst, I2C_ADDR, this->buffer, 2, true);	
}

bool altimeterNEW::validate() {
	// Read factory data and confirm its validity
	this->buffer[0] = c_SET_PROM_READ;
	this->buffer[1] = c_PROM_FACTORY_DATA;
	i2c_write_blocking(this->inst, I2C_ADDR, this->buffer, 2, true);

	//TODO: Valid "Factory Data" is not on datasheet, needs testing to confirm
}
