#pragma once

#include "sensor_i2c.hpp"
#include <Eigen/Dense>

#define MAG_I2C_ADDR 0x30

class Magnetometer : public SensorI2C {
	public:
		//Default constructor, pass I2C instance
		Magnetometer(i2c_inst_t* inst) : SensorI2C{inst, MAG_I2C_ADDR} {};
		
		//Sensor configuration/status check routines
		void initialize();
		bool validate();

        int32_t get_ax() { return ax; }
        int32_t get_ay() { return ay; }
        int32_t get_az() { return az; }

        float scale(int32_t unscaled) { return ((float) unscaled) / S_MAG_SENSITIVITY_FACTOR; }
		
		//Sensor calibration routines
		void calibrateBridgeOffset();

		//Sensor data output
		Eigen::Vector3f getData();	
	
	private:
		//Internal buffer for performing I2C operations
		uint8_t buffer[16];
		
		//Sensor offsets, calculated via performing set/reset measurement routine
		Eigen::Vector3f bridgeOffset = {0, 0, 0};
		
		/* Xout1 - XYZout2 [Output, 18b X/Y/Z as 2-Byte chunks + 1 Byte with (X1:0,Y1:0,Z1:0,0,0)] */
		enum {
			R_MAG_XOUT0 = (0x00),
			R_MAG_XOUT1 = (0x01),
			R_MAG_YOUT0 = (0x02),
			R_MAG_YOUT1 = (0x03),
			R_MAG_ZOUT0 = (0x04),
			R_MAG_ZOUT1 = (0x05),
			R_MAG_XYZOUT2 = (0x06)
		};
		const float S_MAG_SENSITIVITY_FACTOR = 0.0000625f;	//TODO: Confirm G output instead of mG

		/* Tout [Output, 8b Temperature] */
		enum {
			R_MAG_TOUT = (0x07)
		};

		/* Status [State Output, Check for whether Mag/Temp measurements are done] */
		enum {
			R_MAG_STATUS = (0x08),
			b_MAG_STATUS_MEAS_M_DONE = (1 << 0),
			b_MAG_STATUS_MEAS_T_DONE = (1 << 1)
		};

		/* Internal Control 0 [State Configuration, Start measurements and control Set/Reset functionality] */
		enum {
			R_MAG_INTERNAL_CONTROL_0 = (0x09),
			b_MAG_INTERNAL_CONTROL_0_TM_M = (1 << 0),
			b_MAG_INTERNAL_CONTROL_0_TM_T = (1 << 1),
			b_MAG_INTERNAL_CONTROL_0_SET = (1 << 3),
			b_MAG_INTERNAL_CONTROL_0_RESET = (1 << 4)
		};

		/* Internal Control 1 [Configuration, Decimation filter bandwidth] */
		enum {
			R_MAG_INTERNAL_CONTROL_1 = (0x0A),
			B_MAG_INTERNAL_CONTROL_1_FLT_BW_800HZ = (0x03)	//Anything narrower increases latency to outside scope
		};

		/* Internal Control 2 [State Configuration, Continuous measurement frequency/enable CM Mode */
		enum {
			R_MAG_INTERNAL_CONTROL_2 = (0x0B),
			B_MAG_INTERNAL_CONTROL_2_CM_OFF = (0x00),
			B_MAG_INTERNAL_CONTROL_2_CM_FREQ_1000HZ = (0x07),	//Next lowest continuous is 200Hz
			b_MAG_INTERNAL_CONTROL_2_CMM_EN = (1 << 3)
		};

		/* Product ID 1 [Validation] */
		enum {
			R_MAG_PRODUCT_ID = (0x2F),
			B_MAG_PRODUCT_ID_VALUE = (0x30)
		};

        int32_t ax, ay, az;
};
