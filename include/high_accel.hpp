#pragma once

#include "sensor_i2c.hpp"

#define HIGH_I2C_ADDR 0x1D

class high_accel : protected sensor_i2c {
	public: 
		//Default constructor, pass I2C instance
		high_accel(i2c_inst_t* inst);
		
		//Sensor configuration and status check routines
		void initialize() override;
		bool validate() override;
		
		//Sensor data return
		Vector6f getData();

		//Interrupt handling
		void clearInterrupt();

	private:
		/* DEVID [Validation] */
		enum {
			R_ACC_DEVID	= (0x00),
			B_ACC_DEVID_VALUE = (0xE5)
		};
		
		/* THRESH_SHOCK [Configuration, G threshold for single shock interrupt (780mG/LSB)] */
		enum {
			R_ACC_THRESH_SHOCK = (0x1D),
			B_ACC_THRESH_SHOCK_780mG = (0x01),
			B_ACC_THRESH_SHOCK_1560mG = (0x02),
			B_ACC_THRESH_SHOCK_2340mG = (0x03),
			B_ACC_THRESH_SHOCK_3120mG = (0x04),
			B_ACC_THRESH_SHOCK_3900mG = (0x05),
			B_ACC_THRESH_SHOCK_4680mG = (0x06)
		};

		/* DUR [Configuration, duration of single shock interrupt (625us/LSB)] */
		enum {
			R_ACC_DUR = (0x21),
			B_ACC_DUR_625uS = (0x01),
			B_ACC_DUR_1250uS = (0x02),
			B_ACC_DUR_1875uS = (0x03),
			B_ACC_DUR_2500uS = (0x04)
		};

		/* SHOCK_AXES [Configuration, axes used in shock interrupt] */
		enum {
			R_ACC_SHOCK_AXES = (0x2A),
			b_ACC_SHOCK_AXES_X = (1 << 2),
			b_ACC_SHOCK_AXES_Y = (1 << 1),
			b_ACC_SHOCK_AXES_Z = (1 << 0)
		};	

		/* BW_RATE [Configuration, Low Power Mode and ODR] */
		enum {
			R_ACC_BW_RATE = (0x2C),
			b_ACC_LOW_POWER = (1 << 4),
			B_ACC_ODR_100_HZ = (0x0A),
			B_ACC_ODR_400_HZ = (0x0C),
			B_ACC_ODR_800_HZ = (0x0D),
			B_ACC_ODR_3200_HZ = (0x0F)
		};
		
		/* POWER_CTL [State Configuration, Measurement mode (and sleep settings)] */
		enum {
			R_ACC_POWER_CTL = (0x2D),
			b_ACC_POWER_CTL_MEASURE = (1 << 3)
		};

		/* INT_ENABLE [State Configuration, interrupt enabling] */
		enum {
			R_ACC_INT_ENABLE = (0x2E),
			b_ACC_INT_ENABLE_SINGLE_SHOCK = (1 << 6)
		};

		/* INT_MAP [Configuration, Interrupt --> Pin mapping (0 = INT1, 1 = INT2) */
		enum {
			R_ACC_INT_MAP = (0x2F),
			b_ACC_INT_MAP_SINGLE_SHOCK = (1 << 6)
		};

		/* INT_SOURCE [Status, Source of interrupt (read to clear SINGLE_SHOCK interrupt) */
		enum {
			R_ACC_INT_SOURCE = (0x30),
			R_ACC_INT_SOURCE_SINGLE_SHOCK = (1 << 6)
		};

		/* DATA_FORMAT [Configuration, Data output style/formatting (variety of things)] */
		enum {
			R_ACC_DATA_FORMAT = (0x31),
			b_ACC_DATA_FORMAT_SELF_TEST = (1 << 7),	// Self test, adds voltage offset to measurements
			b_ACC_DATA_FORMAT_INT_INVERT = (1 << 5),	// Interrupt output format (0 = Active High, 1 = Active Low)
			b_ACC_DATA_FORMAT_JUSTIFY = (1 << 2),		// Data output format (0 = LSB w/Sign Extension, 1 = MSB)
			B_ACC_DATA_FORMAT_DEFAULT = (0b00001011)	// Default register value with none of above special settings
		};
		
		/* DATAX0-DATAZ1 [Output, 16b data for X/Y/Z (DATA_1 is MSByte, DATA_0 is LSByte) */
		enum {
			R_ACC_DATAX0 = (0x32),
			R_ACC_DATAX1 = (0x33),
			R_ACC_DATAY0 = (0x34),
			R_ACC_DATAY1 = (0x35),
			R_ACC_DATAZ0 = (0x36),
			R_ACC_DATAZ1 = (0x37)
		};
		const float S_ACC_SENSITIVITY_FACTOR = 20.5f;
};

