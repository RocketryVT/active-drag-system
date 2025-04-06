#include "sensor_i2c.hpp"

//DEPRECATED, WILL BE REMOVED FOLLOWING REFACTOR

//Write uint8_t buffer to register block starting at address
void SensorI2C::write_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	//Add the register address to the outgoing I2C TX buffer before writing actual data to the TX buffer, then send
	//i2c_write_byte_raw(this->bus, reg_addr);
	//i2c_write_blocking(this->bus, this->addr, buffer, len, false);

	//TODO: THIS IS A TEMPORARY FIX, REMOVE DYNAMIC MEMORY ALLOCATION DURING SENSOR DRIVER REFACTOR
	uint8_t writeBuffer[len+1];
	writeBuffer[0] = reg_addr;
	for (int i = 0; i < len; i++) writeBuffer[i+1] = buffer[i];
	printf("- SENSORI2C: Performing write operation...\n");
	i2c_write_blocking(this->bus, this->addr, writeBuffer, len+1, false);
	printf("- SENSORI2C: Write operation performed successfully!\n");
}

//Read register block into uint8_t buffer of parameterized len
void SensorI2C::read_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len) {
	i2c_write_blocking(this->bus, this->addr, &reg_addr, 1, true);
	i2c_read_blocking(this->bus, this->addr, buffer, len, false);
}

