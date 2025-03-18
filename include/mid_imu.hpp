#pragma once

#include "sensor_i2c.hpp"

#define IMU_I2C_ADDR 0x68

class mid_imu : public sensor_i2c {
	public: 
		//Default constructor, pass I2C instance
		mid_imu(i2c_inst_t* inst);
		
		//Sensor configuration/status check routines
		void initialize();
		bool validate();
		
		//Sensor data return
		Vector6f getData();
	private:
		/* DEVICE_CONFIG [State Configuration, SPI mode and Reset] */
		enum {
			R_IMU_DEVICE_CONFIG = (0x11),
			b_IMU_DEVICE_CONFIG_SOFT_RESET = (1 << 0)
		};

		/* TEMP_DATA1_UI - TEMP_DATA0_UI [Output, Temperature Sensor] */
	   enum {
	   	   R_IMU_TEMP_DATA1_UI = (0x1D),
		   R_IMU_TEMP_DATA0_UI = (0x1E)
	   };

	   /* ACCEL_DATA_X1_UI - GYRO_DATA_Z0_UI [Output, 3-Axis Gyro and Accel Data */
	   enum {
			R_IMU_ACCEL_DATA_X1_UI = (0x1F),
			R_IMU_ACCEL_DATA_X0_UI = (0x20),
			R_IMU_ACCEL_DATA_Y1_UI = (0x21),
			R_IMU_ACCEL_DATA_Y0_UI = (0x22),
			R_IMU_ACCEL_DATA_Z1_UI = (0x23),
			R_IMU_ACCEL_DATA_Z0_UI = (0x24),
			R_IMU_GYRO_DATA_X1_UI = (0x25),
			R_IMU_GYRO_DATA_X0_UI = (0x26),
			R_IMU_GYRO_DATA_Y1_UI = (0x27),
			R_IMU_GYRO_DATA_Y0_UI = (0x28),
			R_IMU_GYRO_DATA_Z1_UI = (0x29),
			R_IMU_GYRO_DATA_Z0_UI = (0x2A)
	   };
	   const float S_IMU_ACCEL_SENSITIVITY_FACTOR = 1024.0f;
	   const float S_IMU_GYRO_SENSITIVITY_FACTOR = 8.2f;

	   /* PWR_MGMT0 [State Configuration, Sensor enabling] */
	   enum {
		   R_IMU_PWR_MGMT0 = (0x4E),
		   b_IMU_PWR_MGMT0_TEMP_DIS = (1 << 5),
		   B_IMU_PWR_MGMT0_GYRO_MODE_LN = (0x0C),
		   B_IMU_PWR_MGMT0_ACCEL_MODE_LN = (0x03)
	   };

	   /* GYRO_CONFIG0 [Configuration, Sensor FSR/ODR] */
	   enum {
		   R_IMU_GYRO_CONFIG0 = (0x4F),
		   B_IMU_GYRO_CONFIG0_ODR_500HZ = (0x0F)
	   };

	   /* ACCEL_CONFIG0 [Configuration, Sensor FSR/ODR] */
	   enum {
		   R_IMU_ACCEL_CONFIG0 = (0x50),
		   B_IMU_ACCEL_CONFIG0_ODR_500HZ = (0x0f)
	   };

	   /* WHO_AM_I [Validation] */
	   enum {
		   R_IMU_WHO_AM_I = (0x75),
		   B_IMU_WHO_AM_I_VALUE = (0x56)
	   };
};
