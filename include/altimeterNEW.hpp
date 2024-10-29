#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_ADDR (0x77)

/*General Commands*/
enum {
	BARO_RESET = (0x1E),
	BARO_ADC_READ = (0x00)
};

/*PROM Read Commands*/
enum {
	BARO_PROM_READ_FACTORY = (0xA0),
	BARO_PROM_READ_C1 = (0xA1),
	BARO_PROM_READ_C2 = (0xA2),
	BARO_PROM_READ_C3 = (0xA3),
	BARO_PROM_READ_C4 = (0xA4),
	BARO_PROM_READ_C5 = (0xA5),
	BARO_PROM_READ_C6 = (0xA6),
	BARO_PROM_READ_SERIAL_CRC = (0xA7)
};

/*Conversion Request Commands (number is OSR)*/
enum {
	BARO_CONVERT_PRES_256 = (0x40),
	BARO_CONVERT_PRES_4096 = (0x48),
	BARO_CONVERT_TEMP_256 = (0x50),
	BARO_CONVERT_TEMP_4096 = (0x58)
};

//TODO: Change definitions so this class steals the moniker of [altimeter] and deprecate old one
class altimeterNEW {
	private:
		i2c_inst_t* inst;
		uint8_t readBuffer[3];
		
		//Calibration coefficients, loaded during sensor initialization
		uint16_t c1;	//Pressure Sensitivity
		uint16_t c2;	//Pressure Offset
		uint16_t c3;	//Temperature Coefficient of Pressure Sensitivity
		uint16_t c4;	//Temperature Coefficient of Pressure Offset
		uint16_t c5;	//Reference Temperature
		uint16_t c6;	//Temperature Coefficient of the Temperature (???)
	
	public: 
		altimeterNEW(i2c_inst_t* inst);

		void initialize();
		bool validate();

		float getTemperature();
		float getPressure();
		float getAltitude();
};
