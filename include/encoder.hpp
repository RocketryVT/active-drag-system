#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_ADDR 0x06

//Hard definitions will be refactored as tables for full driver

class encoder {
	private:
		i2c_inst_t* inst;
		uint8_t buffer[4];

	public: 
		encoder(i2c_inst_t* inst);

		void initialize();

		bool validate();
};
