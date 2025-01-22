#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define HIGH_I2C_ADDR 0x1D

//Hard definitions will be refactored as tables for full driver
#define c_READ_DEVID 0x00
#define v_DEVID_RESPONSE 0xE5

class high_imu {
	private:
		i2c_inst_t* inst;
		uint8_t buffer[4];

	public: 
		high_imu(i2c_inst_t* inst);

		void initialize();

		bool validate();
};
