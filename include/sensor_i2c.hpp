#pragma once

#include <stdint.h>
#include <hardware/i2c.h>
#include <pico/time.h>

#include <Eigen/Dense>
using namespace Eigen;

//TODO: Figure out better place to define this typedef, only used for IMU return currently
typedef Matrix<float, 6, 1> Vector6f;

class sensor_i2c {
	protected:
		// I2C Internal state, set by configureI2C()
		i2c_inst_t* bus;	//Pico SDK I2C bus instance
		uint8_t bus_addr;	//Sensor address
		void configureI2C(i2c_inst_t* i2c, uint8_t addr);	//Pass the I2C bus instance to sensor, configure address

		// Helper methods for handling common I2C bus R/W patterns
		void write_register_byte(const uint8_t reg, const uint8_t byte);	//Write single byte to register
		uint8_t read_register_byte(const uint8_t reg);						//Read register, return raw byte
		bool check_register_bit(const uint8_t reg, const int bit);			//Read register, return bit (0 = LSB)
		bool check_register_byte(const uint8_t reg, const uint8_t byte);	//Read register, check if byte matches
		uint8_t* read_register_buffer(const uint8_t reg, int numBytes);	//Read multiple registers, return array
		
		//Write a single bit to a register to start a process, check it until it completes, and print the op time (ms)
		void test_print_operation_time(const uint8_t reg, const uint8_t mask);
		
	public:
		// Virtual methods for handling initial sensor configuration
		virtual void initialize() = 0;	//Required override, run all initialization routines and validate
		virtual bool validate() = 0;	//Required override, read some DEVID/WHOAMI/Other register to confirm operation
		
		//TODO: Figure out virtual methods for real-time output updating			
};
