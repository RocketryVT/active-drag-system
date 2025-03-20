#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hardware/i2c.h"

class SensorI2C {
	protected:
		//I2C bus instance and address defining, used to remove the annoyance of passing them each time
		i2c_inst_t* i2c;	//Pico SDK I2C bus instance
		uint8_t addr;		//Sensor address

	public:

        SensorI2C(i2c_inst_t* i2c, uint8_t addr) : i2c { i2c }, addr { addr } {};
		//Helper functions for handling I2C R/W operations
		void write_register(uint8_t reg_addr, uint8_t* buffer, size_t len);
		void read_register(uint8_t reg_addr, uint8_t* buffer, size_t len);
};
