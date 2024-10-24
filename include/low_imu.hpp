#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_ADDR 0x6A

//Hard definitions will be refactored as tables for full driver
#define c_READ_WHOAMI 0x0F
#define v_WHOAMI_RESPONSE 0x70

class low_imu {
	private:
		i2c_inst_t* inst;
		uint8_t buffer[4];

	public: 
		low_imu(i2c_inst_t* inst);

		void initialize();

		bool validate();
};
