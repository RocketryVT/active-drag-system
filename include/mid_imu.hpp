#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

#include "sensor_i2c.hpp"

#define MID_I2C_ADDR 0xD0

//Hard definitions will be refactored as tables for full driver
#define c_READ_WHOAMI 0x75
#define v_WHOAMI_RESPONSE 0x56

class mid_imu : sensor_i2c {
	private:
		i2c_inst_t* inst;
		uint8_t buffer[4];

	public: 
		mid_imu(i2c_inst_t* inst);

		void initialize();

		bool validate();
};
