#include "altimeterNEW.hpp"

//TODO: Update class definitions so this class steals the moniker of [altimeter] and old class is deprecated
altimeterNEW::altimeterNEW(i2c_inst_t* inst) {
	this->inst = inst;
}

void altimeterNEW::initialize() {
	//There are literally only 5 commands for this sensor, no write commands beyond the register are needed
	uint8_t command;

	// Send reset command to sensor
	command = BARO_RESET;
	i2c_write_blocking(this->inst, I2C_ADDR, &commandReg, 1, true);	

	//Load calibration constants into memory for conversions
	command = BARO_PROM_READ_C1;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, this->readBuffer, 2, false);
	c1 = this->readBuffer[1] << 8 | this->readBuffer[0];
	
	command = BARO_PROM_READ_C2;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, this->readBuffer, 2, false);
	c2 = this->readBuffer[1] << 8 | this->readBuffer[0];
	
	command = BARO_PROM_READ_C3;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, this->readBuffer, 2, false);
	c3 = this->readBuffer[1] << 8 | this->readBuffer[0];
	
	command = BARO_PROM_READ_C4;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, this->readBuffer, 2, false);
	c4 = this->readBuffer[1] << 8 | this->readBuffer[0];
	
	command = BARO_PROM_READ_C5;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, this->readBuffer, 2, false);
	c5 = this->readBuffer[1] << 8 | this->readBuffer[0];
	
	command = BARO_PROM_READ_C6;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, this->readBuffer, 2, false);
	c6 = this->readBuffer[1] << 8 | this->readBuffer[0];	
}

bool altimeterNEW::validate() {
	// Read factory data and confirm its validity
	this->buffer[0] = c_SET_PROM_READ;
	this->buffer[1] = c_PROM_FACTORY_DATA;
	i2c_write_blocking(this->inst, I2C_ADDR, this->buffer, 2, true);

	//TODO: Valid "Factory Data" is not on datasheet, needs testing to confirm
}
