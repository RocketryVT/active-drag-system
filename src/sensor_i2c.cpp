#include "sensor_i2c.hpp"

//DEPRECATED - TODO: Discuss removal of SensorI2C as its scope has shrunk to almost nothing

//Write uint8_t buffer to i2c with internally configured I2C instance
void SensorI2C::write_buffer(uint8_t* buffer, size_t len, bool nostop) {
	//Passthrough internal fields to sdk method to minimize redundant parameters
	i2c_write_blocking(this->bus, this->addr, buffer, len, nostop);
}

//TEMPORARY, WILL BE REMOVED AFTER REFACTOR: Overloaded write_buffer to fix compilation of deprecated drivers
void SensorI2C::write_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	//Use the botched index shift strategy with dynamic buffer allocation (ew, I know) 
	uint8_t writeBuffer[len+1];
	writeBuffer[0] = reg_addr;
	for (int i = 0; i < len; i++) writeBuffer[i+1] = buffer[i];
	i2c_write_blocking(this->bus, this->addr, writeBuffer, len+1, false);
}

//Read register block into uint8_t buffer of parameterized len
void SensorI2C::read_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	i2c_write_blocking(this->bus, this->addr, &reg_addr, 1, true);
	i2c_read_blocking(this->bus, this->addr, buffer, len, false);
}

