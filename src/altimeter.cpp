#include "altimeter.hpp"

//Default constructor, pass I2C instance
altimeter::altimeter(i2c_inst_t* inst) {
	this->inst = inst;
}

bool altimeter::initialize() {
	//There are literally only 5 commands for this sensor, no write commands beyond the register are needed
	uint8_t command;

	// Send reset command to sensor
	command = BARO_RESET;
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);	
	
	//Read PROM values into memory - 8x16bit registers
	command = BARO_PROM_READ_START;
	uint8_t promBuffer[16];	
	i2c_write_blocking(this->inst, BARO_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, BARO_I2C_ADDR, &command, 16, false);
	
	//Check factory ID to verify that values made it through successfully
	uint16_t promBuffer[0] | promBuffer[1];
	//TODO: The datasheet doesn't actually say what the "factory data" is,
	// we'll have to confirm it experimentally for this status check

	//Load calibration constants into memory for conversions
	c1 = this->promBuffer[2] << 8 | this->promBuffer[3];
	c2 = this->promBuffer[4] << 8 | this->promBuffer[5];
	c3 = this->promBuffer[6] << 8 | this->promBuffer[7];
	c4 = this->promBuffer[8] << 8 | this->promBuffer[9];
	c5 = this->promBuffer[10] << 8 | this->promBuffer[11];
	c6 = this->promBuffer[12] << 8 | this->promBuffer[13];	

	//TODO: Load CRC, need to figure out how this works
	
	//Read temperature value and use it to set whether temperature compensation is needed
	//Note: This WILL lock the bus with read requests until temperature is done converting
	forceUpdateTemperature();

	//TODO: Figure out control flow of feeding asynchronous data to filter, for now start a conversion on startup
	requestPressureConversion();
	//Assume sensor is being pinged by the filter every cycle for isFreshAltAvailable() after this point
}

//Check that sensor is reporting correctly to the spec
bool altimeter::validate() {
	// Read factory data and confirm its validity
	uint8_t command = BARO_PROM_READ_START;
	uint8_t readBuffer[2];
	i2c_write_blocking(this->inst, I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, I2C_ADDR, readBuffer, 2, false);

	//TODO: See initialize(), "Factory Data" is not on datasheet
	return true;

	//TODO: Implement CRC validation as well for data returns

}

//Return altitude (m) as a float 
float altimeter::getAltitude() {
	return altitude;
}

//Return the pressure value (mBar) as a float 
float altimeter::getPressure() {
	return pressure;
}

//Return the temperature (C) as a float
float altimeter::getTemperature() {
	return temperature;
}

//Request a new pressure (d1) ADC conversion at configured OSR
void altimeter::requestPressureConversion() {
	uint8_t command = BARO_CONVERT_PRES_256;	//OSR 256
	i2c_write_blocking(this->inst, BARO_I2C_ADDR, &command, 1, false);
}	

//Check conversion status of ADC register, calculate altitude and request a new conversion if data freshly available
bool isFreshAltAvailable() {
	//Assume that pressure conversion is underway, shouldn't ever fetch temperature based on driver implementation
	uint32_t readValue = getADC();
	if (readValue == 0) {
		//No new altitude is available
		return false;
	} else {
		d1 = readValue;	//Pass measured conversion to internal variable
		calculateAltitude();	//Calculate new altitude with digital measurement and temperature offsets

		//New altitude is available for filtering
		return true;
	}
}

//Take D1 value in memory and use it to compute altitude and pressure, then store them
void calculateAltitude() {
	//Calculate constants used for pressure computation
	off = (c2 << 16) + ((c4 * dT) >> 7);	//Update pressure offset from temperature
	sens = (c1 << 15) + ((c3 * dT) >> 8);	//Update pressure sensitivity from temperature

	//Add offsets to the above if low temperature compensation is enabled
	if (compensatingTemperature) {
		off -= (5 * std::pow(dTem - 2000, 2)) >> 1;
		sens -= (5 * std::pow(dTem - 2000, 2)) >> 2;
	}

	//Calculate pressure using the freshly calculated temperature offsets and stored d1
	int32_t p = (((d1 * sens) >> 21) - off) >> 15;

	//Divide the integer value down two decimal places and store 
	pressure = (float)p / 100.0f;	//Integer pressure has 2-decimal precision as LSD and LSD+1

	//TODO: Update to add altitude calculation, this will need its own input sea level pressure
}

//Take D2 value in memory and use it to compute temperature value, then store it
void calculateTemperature() {
	//Calculate the temperature using formula from datasheet
	dt = d2 - (c5 << 8);			//Update dT
	dTem = (2000 + dT * c6) >> 23;	//Use dT to calculate int temp

	//Use digital temp value to set flag for whether temperature compensation is needed
	compensatingTemperature = (dTem / 100 < LOW_TEMP_THRESHOLD) ? true : false;

	//With flag set, do compensation if necessary
	if (compensatingTemperature) dTem -= (dT >> 31);
	
	//Convert the integer temperature to float temperature, and update class variable
	temperature = (float)temp / 100.0f;		//Integer temp has 2-decimal precision as LSD and LSD+1
}

//Request and return raw ADC value, if output is 0 data is not ready/requested
uint32_t altimeter::getADC() {
	//Run ADC read command
	uint8_t command = BARO_ADC_READ;
	uint8_t readBuffer[4];						//32-bit digital output value
	i2c_write_blocking(this->i2c, BARO_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->i2c, BARO_I2C_ADDR, readBuffer, 4, false);	
	
	//Return combined 32 bit value
	return readBuffer[3] << 24 | readBuffer[2] << 16 | readBuffer[1] << 8 | readBuffer[0];
}
//Request temperature and block thread until it is complete, and update temperature compensation flag
void forceUpdateTemperature() {
	uint8_t command;

	//Write a temperature conversion request
	command = BARO_COVERT_TEMP_4096;	//Largest OSR
	uint8_t readBuffer[4];				//32-bit temperature value
	i2c_write_blocking(this->inst, BARO_I2C_ADDR, &command, 1, true);
	i2c_read_blocking(this->inst, BARO_I2C_ADDR, &command, 4, false);

	//Run the ADC read command until temperature is available to be converted
	uint32_t tempD1 = 0;
	while (tempD1 == 0) {
		tempD1 = getADC();
		//TODO: Add reasonable delay to avoid saturating the I2C bus or confusing sensor
	}
	d1 = tempD1;

	//Calculate new temperature value
	calculateTemperature();

	//TODO: As this is intended to be run only occasionally, maybe add some simple in-driver filtering?
	// Alternative is periodically running a temperature refresh as well, needs testing for drift w/low temp
}



