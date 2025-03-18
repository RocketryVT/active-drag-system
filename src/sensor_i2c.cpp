#include "sensor_i2c.hpp"

//Write single uint8_t byte to register at address
void sensor_i2c::write_register_byte(uint8_t reg_addr, uint8_t byte) {
	uint8_t writeBuffer[2] = {reg_addr, byte};
	i2c_write_blocking(this->bus, this->bus_addr, writeBuffer, 2, false);	
}

//Write uint8_t buffer to register block starting at address
void sensor_i2c::write_register_buffer(uint8_t reg_addr, uint8_t* dataBuffer, size_t len) {
	i2c_write_blocking(this->bus, this->bus_addr, &reg_addr, 1, true);
	i2c_write_blocking(this->bus, this->bus_addr, dataBuffer, len, false);
}

//Read register block into uint8_t buffer of parameterized len
void sensor_i2c::read_register_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	i2c_write_blocking(this->bus, this->bus_addr, &reg_addr, 1, true);
	i2c_read_blocking(this->bus, this->bus_addr, buffer, len, false);
}

//Read register, return bit at [bit] index (0 = LSB)
bool sensor_i2c::check_register_bit(uint8_t reg_addr, const int bit) {
	uint8_t readBuffer[1] = {0};
	read_register_buffer(reg_addr, readBuffer, 1);
	return (readBuffer[0] >> bit) & 1;
}

//Read register, return whether byte returned matches the expected byte (parameter)
bool sensor_i2c::check_register_byte(uint8_t reg_addr, const uint8_t byte) {
	uint8_t readBuffer[1] = {0};
	read_register_buffer(reg_addr, readBuffer, 1);
	return (readBuffer[0] == byte);
}


