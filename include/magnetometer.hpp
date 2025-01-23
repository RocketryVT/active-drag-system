#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define MAG_I2C_ADDR 0x30

//Hard definitions will be refactored as tables for full driver
#define c_READ_PRODID 0x2F
#define v_PRODID_RESPONSE 0x30

class magnetometer {
	private:
		i2c_inst* inst;
		uint8_t buffer[4];

	public:
		magnetometer(i2c_inst_t* inst);

		void initialize();

		bool validate();

};
