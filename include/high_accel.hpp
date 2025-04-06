#pragma once
#include "sensor_i2c.hpp"

#define HIGH_I2C_ADDR 0x1D

// DEVID [Validation]
#define R_ACC_DEVID 		0x00
#define B_ACC_DEVID_VALUE 	0xE5
		
// THRESH_SHOCK [Configuration, G threshold for single shock interrupt (780mG/LSB)]
#define R_ACC_THRESH_SHOCK 			0x1D
#define B_ACC_THRESH_SHOCK_780mG 	0x01
#define B_ACC_THRESH_SHOCK_1560mG 	0x02
#define B_ACC_THRESH_SHOCK_2340mG	0x03
#define B_ACC_THRESH_SHOCK_3120mG	0x04
#define B_ACC_THRESH_SHOCK_3900mG	0x05
#define B_ACC_THRESH_SHOCK_4680mG	0x06

// DUR [Configuration, duration of single shock interrupt (625us/LSB)]
#define R_ACC_DUR 			0x21
#define B_ACC_DUR_625uS 	0x01
#define B_ACC_DUR_1250uS 	0x02
#define B_ACC_DUR_1875uS 	0x03
#define B_ACC_DUR_2500uS 	0x04

// SHOCK_AXES [Configuration, axes used in shock interrupt]
#define R_ACC_SHOCK_AXES 0x2A
typedef union {
	struct {
		bool SHOCK_Z :1;		//Z-Axis shock interrupt enable
		bool SHOCK_Y :1;		//Y-Axis shock interrupt enable
		bool SHOCK_X :1;		//X-Axis shock interrupt enable
		bool SUPPRESS :1;		//Suppresses double shock detection
		uint8_t RESERVED :4;	//Reserved
	} fields;
	uint8_t data;	
} ACC_SHOCK_AXES;
#define b_ACC_SHOCK_AXES_SET_ENABLED 	0b1
#define b_ACC_SHOCK_AXES_SET_DISABLED 	0b0

// BW_RATE [Configuration, Low Power Mode and ODR]
#define R_ACC_BW_RATE 0x2C
typedef union {
	struct {
		uint8_t RATE :4;		//ODR configuration
		bool LOW_POWER :1;		//Low-power/Normal mode configuration
		uint8_t RESERVED :3;	//Reserved
	} fields;
	uint8_t data;
} ACC_BW_RATE;
#define B_ACC_ODR_100_HZ 				0x0A
#define B_ACC_ODR_400_HZ 				0x0C
#define B_ACC_ODR_800_HZ 				0x0D
#define B_ACC_ODR_3200_HZ 				0x0F
#define b_ACC_BW_RATE_LOW_POWER_MODE 	0b1
#define b_ACC_BW_RATE_NORMAL_POWER_MODE 0b0
		
// POWER_CTL [State Configuration, Measurement mode (and sleep settings)]
#define R_ACC_POWER_CTL 0x2D
typedef union {
	struct {
		uint8_t WAKEUP :2;	//Sampling rate control during sleep mode
		bool SLEEP :1;		//Sleep mode configuration
		bool MEASURE :1;	//Measurement/Standby mode configuration
		bool AUTO_SLEEP :1;	//Auto sleep function enable
		bool LINK :1;		//Link activity and inactivity functions
		bool RESERVED :1;	//Reserved
	} fields;
	uint8_t data;
} ACC_POWER_CTL;
#define b_ACC_POWER_CTL_MEASURE_MODE 0x01
#define b_ACC_POWER_CTL_STANDBY_MODE 0x00

//TODO: All interrupt configuration registers use the same mapping, figure out a way to consolidate bitfields
// INT_ENABLE [State Configuration, interrupt enabling]
#define R_ACC_INT_ENABLE 0x2E
typedef union {
	struct {
		bool OVERRUN :1;		//FIFO buffer overwrite interrupt enable
		bool WATERMARK :1;		//??? TODO: Figure out watermark bits
		bool RESERVED :1;		//Reserved
		bool INACTIVITY :1;		//Inactivity interrupt enable
		bool ACTIVITY :1;		//Activity interrupt enable
		bool DOUBLE_SHOCK :1;	//Double shock interrupt enable
		bool SINGLE_SHOCK :1;	//Single shock interrupt enable
		bool DATA_READY :1;		//FIFO buffer new data interrupt enable
	} fields;
	uint8_t data;
} ACC_INT_ENABLE;
#define b_ACC_INT_ENABLE_SET_ENABLED  0b1
#define b_ACC_INT_ENABLE_SET_DISABLED 0b0

// INT_MAP [Configuration, Interrupt --> Pin mapping (0 = INT1, 1 = INT2) */
#define R_ACC_INT_MAP 0x2F
typedef union {
	struct {
		bool OVERRUN :1;		//FIFO buffer overwrite interrupt enable
		bool WATERMARK :1;		//??? TODO: Figure out watermark bits
		bool RESERVED :1;		//Reserved
		bool INACTIVITY :1;		//Inactivity interrupt enable
		bool ACTIVITY :1;		//Activity interrupt enable
		bool DOUBLE_SHOCK :1;	//Double shock interrupt enable
		bool SINGLE_SHOCK :1;	//Single shock interrupt enable
		bool DATA_READY :1;		//FIFO buffer new data interrupt enable
	} fields;
	uint8_t data;
} ACC_INT_MAP;
#define b_ACC_INT_MAP_LINK_INTERRUPT_INT1 0b0
#define b_ACC_INT_MAP_LINK_INTERRUPT_INT2 0b1

// INT_SOURCE [Status, Source of interrupt (read to clear SINGLE_SHOCK interrupt) */
#define R_ACC_INT_SOURCE 0x30
typedef union {
	struct {
		bool OVERRUN :1;		//FIFO buffer overwrite interrupt enable
		bool WATERMARK :1;		//??? TODO: Figure out watermark bits
		bool RESERVED :1;		//Reserved
		bool INACTIVITY :1;		//Inactivity interrupt enable
		bool ACTIVITY :1;		//Activity interrupt enable
		bool DOUBLE_SHOCK :1;	//Double shock interrupt enable
		bool SINGLE_SHOCK :1;	//Single shock interrupt enable
		bool DATA_READY :1;		//FIFO buffer new data interrupt enable
	} fields;
	uint8_t data;
} ACC_INT_SOURCE;

// DATA_FORMAT [Configuration, Data output style/formatting (variety of things)] */
#define R_ACC_DATA_FORMAT 0x31
typedef union {
	struct {
		uint8_t RESERVED :2;	//Reserved
		bool JUSTIFY :1;		//Left(MSB)/Right(LSB, Sign extended) output justification
		uint8_t RESERVED_ :2;	//Reserved
		bool INT_INVERT :1;		//Interrupt pin polarity configuration
		bool SPI :1;			//SPI 3/4 wire mode configuration
		bool SELF_TEST :1;		//Configure self-test mode
	} fields;
	uint8_t data;
} ACC_DATA_FORMAT;
#define b_ACC_DATA_FORMAT_JUSTIFY_RIGHT 		0b0
#define b_ACC_DATA_FORMAT_JUSTIFY_LEFT 			0b1
#define b_ACC_DATA_FORMAT_INTERRUPT_ACTIVE_HIGH 0b0
#define b_ACC_DATA_FORMAT_INTERRUPT_ACTIVE_LOW 	0b1
#define b_ACC_DATA_FORMAT_ENABLE_SELF_TEST 		0b1
#define b_ACC_DATA_FORMAT_DISABLE_SELF_TEST 	0b0
		
// DATAX0-DATAZ1 [Output, 16b data for X/Y/Z (DATA_1 is MSByte, DATA_0 is LSByte)
#define R_ACC_DATAX0 0x32
#define R_ACC_DATAX1 0x33
#define R_ACC_DATAY0 0x34
#define R_ACC_DATAY1 0x35
#define R_ACC_DATAZ0 0x36
#define R_ACC_DATAZ1 0x37
#define S_ACC_SENSITIVITY_FACTOR 20.5f

class HighAccel : public SensorI2C {
	public: 
		//Default constructor, pass I2C instance
		HighAccel(i2c_inst_t* inst);
		
		//Sensor configuration and status check routines
		void initialize();
		bool validate();
		
		//Sensor data return
		void update();		//Request data from sensor over I2C and format it
		Vector3f getData();	//Return formatted data members as a vector
		float getX();		//Return formatted x acceleration
		float getY();		//Return formatted y acceleration
		float getZ();		//Return formatted z acceleration

		//Interrupt handling
		void clearInterrupt();

	private:
		//Internal data members, stored as an Eigen vector
		Vector3f data;
		
		//Internal buffer and instance for performing I2C operations
		uint8_t buffer[16];

		//Internal register fields for configuration
		ACC_SHOCK_AXES shock_axes;
		ACC_BW_RATE bw_rate;
		ACC_POWER_CTL power_ctl;
		ACC_INT_ENABLE int_enable;
		ACC_INT_MAP	int_map;
		ACC_INT_SOURCE int_source;
		ACC_DATA_FORMAT data_format;
};
