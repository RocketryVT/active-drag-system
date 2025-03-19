#include "sensor_i2c.hpp"

//Write uint8_t buffer to register block starting at address
void SensorI2C::write_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	i2c_write_blocking(this->bus, this->bus_addr, &reg_addr, 1, true);
	i2c_write_blocking(this->bus, this->bus_addr, buffer, len, false);
}

//Read register block into uint8_t buffer of parameterized len
void SensorI2C::read_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	i2c_write_blocking(this->bus, this->bus_addr, &reg_addr, 1, true);
	i2c_read_blocking(this->bus, this->bus_addr, buffer, len, false);
}

