#pragma once

#include "boards/pico.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

//TODO: Figure out better place to define these, shouldn't be inside sensor_i2c scope
#include <Eigen/Dense>
using namespace Eigen;
typedef Matrix<float, 6, 1> Vector6f;

class sensor_i2c {
	protected:
		//I2C bus instance and address defining, used to remove the annoyance of passing them each time
		i2c_inst_t* bus;	//Pico SDK I2C bus instance
		uint8_t bus_addr;	//Sensor address
	public:
		//Helper functions for handling I2C R/W operations
		void write_register_byte(uint8_t reg_addr, uint8_t byte);
		void write_register_buffer(uint8_t reg_addr, uint8_t* dataBuffer, size_t len);
		void read_register_buffer(uint8_t reg_addr, uint8_t* data, size_t len);
		
		//Helper functions for processing common I2C R/W patterns
		bool check_register_bit(uint8_t reg_addr, const int bit);			//Read register, return bit (0 = LSB)
		bool check_register_byte(uint8_t reg_addr, const uint8_t byte);	//Read register, check if byte matches
};
