#include "sensor_i2c.hpp"

//Write single byte to register
void sensor_i2c::write_register_byte(const uint8_t reg, const uint8_t byte) {
	uint8_t writeBuffer[2] = {reg, byte};
	i2c_write_blocking(bus, bus_addr, writeBuffer, 2, false);
}

//Read register, return raw byte
uint8_t sensor_i2c::read_register_byte(const uint8_t reg) {
	uint8_t readBuffer[1];
	i2c_write_blocking(bus, bus_addr, &reg, 1, true);
	i2c_read_blocking(bus, bus_addr, readBuffer, 1, false);
	return readBuffer[0];
}

//Read register, return bit at [bit] index (0 = LSB)
bool sensor_i2c::check_register_bit(const uint8_t reg, const int bit) {
	return (read_register_byte(reg) >> bit) & 1;
}

//Read register, return whether byte returned matches the expected byte (parameter)
bool sensor_i2c::check_register_byte(const uint8_t reg, const uint8_t byte) {
	uint8_t readByte = read_register_byte(reg);
	return (readByte == byte);
}

//Read multiple registers into passed array with parameterized size (of length numBytes, NO ERROR CHECKING)
void sensor_i2c::read_register_buffer_into_array(const uint8_t reg, uint8_t buffer[], const int numBytes) {
	for (int i = 0; i < numBytes; i++) buffer[i] = 0;	//Overkill but just making sure
	i2c_write_blocking(bus, bus_addr, &reg, 1, true);
	i2c_read_blocking(bus, bus_addr, buffer, numBytes, false);
}

//Write a single bit to a register to start a process, check it until it completes, and print the op time (ms)
void test_print_operation_time(const uint8_t reg, const uint8_t mask) {
	//TODO: Figure out how to reasonably integrate the pico time module into this
}

//Pass the I2C bus instance to the sensor, and register its address internally (MUST BE CALLED IN CONSTRUCTOR)
void sensor_i2c::configureI2C(i2c_inst_t* i2c, uint8_t addr) {
	this->bus = i2c;
	this->bus_addr = addr;
}
