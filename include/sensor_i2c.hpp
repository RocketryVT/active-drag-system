#pragma once

#include "boards/pico.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

//TODO: Figure out better place to define these, shouldn't be inside SensorI2C scope
#include <Eigen/Dense>
using namespace Eigen;
typedef Matrix<float, 6, 1> Vector6f;

class SensorI2C {
	protected:
		//I2C bus instance and address defining, used to remove the annoyance of passing them each time
		i2c_inst_t* bus;	//Pico SDK I2C bus instance
		uint8_t bus_addr;	//Sensor address

	public:
		//Helper functions for handling I2C R/W operations
		void write_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len);
		void read_buffer(uint8_t reg_addr, uint8_t* buffer, size_t len);
};
