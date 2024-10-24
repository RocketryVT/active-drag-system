#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_ADDR 0x77

//Hard definitions will be refactored as tables for full driver
#define c_RESET 0x1E
#define c_SET_PROM_READ 0xA6
#define c_PROM_FACTORY_DATA 0xA0

//TODO: Change definitions so this class steals the moniker of [altimeter] and deprecate old one
class altimeterNEW {
	private:
		i2c_inst_t* inst;

	public: 
		altimeterNEW(i2c_inst_t* inst);

		void initialize();

		bool validate();
};
