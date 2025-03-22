#include "high_accel.hpp"
#include "hardware/i2c.h"

//Run initialization and reset routines
void HighAccel::initialize() {
	//TODO: Refactor to consolidate into multi-register buffer writes, organize by register order and clump

	//Configure power mode and ODR
	buffer[0] = R_ACC_BW_RATE;
		bw_rate.fields.LOW_POWER = b_ACC_BW_RATE_NORMAL_POWER_MODE;
		bw_rate.fields.RATE = B_ACC_ODR_800_HZ;
		buffer[1] = bw_rate.data;
	write_buffer(buffer, 2, false);
	
	//Configure data output format
	read_buffer(R_ACC_DATA_FORMAT, buffer, 1);
		data_format.data = buffer[0];	//Some reserved fields are set to 1, read byte and *then* set desired bits
	buffer[0] = R_ACC_DATA_FORMAT;
		data_format.fields.JUSTIFY = b_ACC_DATA_FORMAT_JUSTIFY_RIGHT;
		data_format.fields.INT_INVERT = b_ACC_DATA_FORMAT_INTERRUPT_ACTIVE_HIGH;
		buffer[1] = data_format.data;
	write_buffer(buffer, 2, false);

	//Configure interrupt settings and mapping
	buffer[0] = R_ACC_THRESH_SHOCK;
		buffer[1] = B_ACC_THRESH_SHOCK_1560mG;
		write_buffer(buffer, 2, false);
	buffer[0] = R_ACC_DUR;
		buffer[1] = B_ACC_DUR_2500uS;
		write_buffer(buffer, 2, false);
	buffer[0] = R_ACC_SHOCK_AXES;
		shock_axes.fields.SHOCK_Z = b_ACC_SHOCK_AXES_SET_ENABLED;
		buffer[1] = shock_axes.data;
		write_buffer(buffer, 2, false);
	buffer[0] = R_ACC_INT_MAP;
		int_map.fields.SINGLE_SHOCK = b_ACC_INT_MAP_LINK_INTERRUPT_INT2;	//TODO: Confirm this mapping in testing
		buffer[1] = int_map.data;
		write_buffer(buffer, 2, false);
	
	//Enable interrupts
	buffer[0] = R_ACC_INT_ENABLE;
		int_enable.fields.SINGLE_SHOCK = b_ACC_INT_ENABLE_SET_ENABLED;
		write_buffer(buffer, 2, false);

	//Set to measurement mode
	buffer[0] = R_ACC_POWER_CTL;
		power_ctl.fields.MEASURE = b_ACC_POWER_CTL_MEASURE_MODE;
		buffer[1] = power_ctl.data;
		write_buffer(R_ACC_POWER_CTL, buffer, 1);
}

//Read devid register and confirm its validity
bool HighAccel::validate() {
	//Check the DEVID register and confirm its response
	read_buffer(R_ACC_DEVID, buffer, 1);
	return (buffer[0] == B_ACC_DEVID_VALUE);
}

//Read all six data registers in one operation, format them, and set internal members to the converted values
void HighAccel::update() {
	//Read DATAX0 - DATAZ1 as a block
	read_buffer(R_ACC_DATAX0, buffer, 6);	//TODO: Confirm this formatting works
	
	//Split buffer into individual fields
	int16_t int_x, int_y, int_z = 0;
	int_x = ((int16_t) buffer[0]) | ((int16_t) buffer[1] << 8);
	int_y = ((int16_t) buffer[2]) | ((int16_t) buffer[3] << 8);
	int_z = ((int16_t) buffer[4]) | ((int16_t) buffer[5] << 8);
	
	//Divide by scale factor to format as float for output
	data[0] = ((float) int_x) / S_ACC_SENSITIVITY_FACTOR;
	data[1] = ((float) int_y) / S_ACC_SENSITIVITY_FACTOR;
	data[2] = ((float) int_z) / S_ACC_SENSITIVITY_FACTOR;
}

Vector3f HighAccel::get_data() {
	return data;
}

float HighAccel::getX() {
	return data[0];
}

float HighAccel::getY() {
	return data[1];
}

float HighAccel::getZ() {
	return data[2];
}

/*
//Read interrupt source register to clear interrupt
void high_accel::clearInterrupt() {
	uint8_t intSource = read_register_buffer(R_ACC_INT_SOURCE);
}
*/
