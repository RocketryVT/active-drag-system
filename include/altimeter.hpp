#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "sensor_i2c.hpp"

#define BARO_I2C_ADDR (0x77)
#define LOW_TEMP_THRESHOLD 20	//Temperature in C below which compensation is needed in digital conversion

/*General Commands*/
enum {
	BARO_RESET = (0x1E),
	BARO_ADC_READ = (0x00)
};

/*PROM Read Commands*/
enum {
	BARO_PROM_READ_START = (0xA0),
	BARO_PROM_READ_C1 = (0xA2),
	BARO_PROM_READ_C2 = (0xA4),
	BARO_PROM_READ_C3 = (0xA6),
	BARO_PROM_READ_C4 = (0xA8),
	BARO_PROM_READ_C5 = (0xAA),
	BARO_PROM_READ_C6 = (0xAC),
	BARO_PROM_READ_SERIAL_CRC = (0xAE)
};

/*Conversion Request Commands (number is OSR)*/
enum {
	BARO_CONVERT_PRES_256 = (0x40),
	BARO_CONVERT_PRES_512 = (0x42),
	BARO_CONVERT_PRES_1024 = (0x44),
	BARO_CONVERT_PRES_2048 = (0x46),
	BARO_CONVERT_PRES_4096 = (0x48),
	BARO_CONVERT_TEMP_256 = (0x50),
	BARO_CONVERT_TEMP_512 = (0x52),
	BARO_CONVERT_TEMP_1024 = (0x54),
	BARO_CONVERT_TEMP_2048 = (0x56),
	BARO_CONVERT_TEMP_4096 = (0x58)
};

class altimeter : protected sensor_i2c {
	private:
		
		//Calibration coefficients, loaded during sensor initialization
		uint16_t c1;	//Pressure Sensitivity
		uint16_t c2;	//Pressure Offset
		uint16_t c3;	//Temperature Coefficient of Pressure Sensitivity
		uint16_t c4;	//Temperature Coefficient of Pressure Offset
		uint16_t c5;	//Reference Temperature
		uint16_t c6;	//Temperature Coefficient of the Temperature (???)
		
		//Storage for values used in onboard conversion math
		// Used for decoupling conversion request commands from control loop
		uint32_t d1;	//Digital Pressure Value
		uint32_t d2;	//Digital Temperature Value
		int32_t dT;		//Difference from actual and reference temperature (c5)
		int32_t dTem;	//Digital temperature
		int64_t off;	//Temperature offset for pressure
		int64_t sens;	//Sensitivity at temperature

		//Storage for the actual floating point data, public getters available
		float altitude;
		float pressure;
		float temperature;
		
		//State values for conversion math
		bool compensatingTemperature;

		//Methods for dealing with sensor reading procedure
 		uint32_t getADC();	
	    void calculateAltitude();
	    void calculateTemperature();
	    void requestPressureConversion();

	public: 
		//Default constructor, pass I2C instance
		altimeter(i2c_inst_t* inst);
		
		//Sensor configuration/status check routines
		void initialize() override;
		bool validate() override;
		
		//Sensor polls for data requests
		float getTemperature();
		float getPressure();
		float getAltitude();
		
		//Status checks for data readiness
		bool isFreshAltAvailable();
		void forceUpdateTemperature();		
};
