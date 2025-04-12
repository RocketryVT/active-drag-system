#pragma once

#include <stdint.h>

#include <hardware/i2c.h>
#include <hardware/clocks.h>

#define IIM42653_I2C_ADDR 0x68
#define IIM42653_CLOCK_SOURCE_SYSTEM 0x6    //Use system clock as source
#define IIM42653_CLOCK_DIVISOR 3125         //125MHz --> 40kHz
	
// DEVICE_CONFIG [State Configuration, SPI mode and Reset]
#define R_IIM42653_DEVICE_CONFIG 0x11
typedef union {
    struct {
        bool SOFT_RESET_CONFIG :1;  //Enable/disable soft reset
        uint8_t RESERVED :3;        //Reserved
        bool SPI_MODE :1;           //SPI Mode (internal), 0/3 or 1/2
        uint8_t RESERVED_ :3;       //Reserved
    } fields;
    uint8_t data;
} IIM42653_DEVICE_CONFIG;
#define b_IIM42653_DEVICE_CONFIG_ENABLE_SOFT_RESET  0b1
#define b_IIM42653_DEVICE_CONFIG_DISABLE_SOFT_RESET 0b0

// DRIVE_CONFIG [Configuration, SPI/I2C Speed]
#define R_IIM42653_DRIVE_CONFIG 0x13
typedef union {
    struct {
        uint8_t SPI_SLEW_RATE :3;   //SPI slew rate on Pin 14
        uint8_t I2C_SLEW_RATE :3;   //I2C slew rate on Pin 14
        uint8_t RESERVED :2;        //Reserved
    } fields;
    uint8_t data;
} IIM42653_DRIVE_CONFIG;
#define B_IIM42653_DRIVE_CONFIG_SLEW_20_TO_60_nS 0x00
#define B_IIM42653_DRIVE_CONFIG_SLEW_2_TO_6_nS   0x20

// TEMP_DATA1_UI - TEMP_DATA0_UI [Output, Temperature Sensor]
#define R_IIM42653_TEMP_DATA1_UI 0x1D
#define R_IIM42653_TEMP_DATA0_UI 0x1E

// ACCEL_DATA_X1_UI - GYRO_DATA_Z0_UI [Output, 3-Axis Gyro and Accel Data]
#define R_IIM42653_ACCEL_DATA_X1_UI          0x1F
#define R_IIM42653_ACCEL_DATA_X0_UI          0x20
#define R_IIM42653_ACCEL_DATA_Y1_UI          0x21
#define R_IIM42653_ACCEL_DATA_Y0_UI          0x22
#define R_IIM42653_ACCEL_DATA_Z1_UI          0x23
#define R_IIM42653_ACCEL_DATA_Z0_UI          0x24
#define R_IIM42653_GYRO_DATA_X1_UI           0x25
#define R_IIM42653_GYRO_DATA_X0_UI           0x26
#define R_IIM42653_GYRO_DATA_Y1_UI           0x27
#define R_IIM42653_GYRO_DATA_Y0_UI           0x28
#define R_IIM42653_GYRO_DATA_Z1_UI           0x29
#define R_IIM42653_GYRO_DATA_Z0_UI           0x2A
#define S_IIM42653_ACCEL_SENSITIVITY_FACTOR  1024.0f
#define S_IIM42653_GYRO_SENSITIVITY_FACTOR   8.2f

// PWR_MGMT0 [State Configuration, Sensor enabling]
#define R_IIM42653_PWR_MGMT0 0x4E
typedef union {
	struct {
        uint8_t ACCEL_MODE :2;  //Accelerometer power mode
        uint8_t GYRO_MODE :2;   //Gyroscope power mode
        bool IDLE :1;           //Run RC oscillator even if Accel/Gyro are off
        bool TEMP_DIS :1;       //Disable the temperature sensor
        bool S4S_ENABLE :1;     //Enable synchronous timing control (???) with custom command
        bool RESERVED :1;       //Reserved
    } fields;
    uint8_t data;
} IIM42653_PWR_MGMT0;
#define B_IIM42653_PWR_MGMT0_ACCEL_MODE_OFF       0x00
#define B_IIM42653_PWR_MGMT0_ACCEL_MODE_LOW_NOISE 0x03
#define B_IIM42653_PWR_MGMT0_GYRO_MODE_OFF        0x00
#define B_IIM42653_PWR_MGMT0_GYRO_MODE_LOW_NOISE  0x03
#define B_IIM42653_PWR_MGMT0_TEMP_DISABLE         0b1
#define B_IIM42653_PWR_MGMT0_TEMP_ENABLE          0b0

// GYRO_CONFIG0 [Configuration, Sensor FSR/ODR]
#define R_IIM42653_GYRO_CONFIG0 0x4F
typedef union {
    struct {
        uint8_t GYRO_ODR :4;        //ODR, see datasheet for range
        bool RESERVED :1;           //Reserved
        uint8_t GYRO_UI_FS_SEL :3;  //FSR, see datasheet for range
    } fields;
    uint8_t data;
} IIM42653_GYRO_CONFIG0;
#define B_IIM42653_GYRO_CONFIG0_ODR_100HZ   0x08
#define B_IIM42653_GYRO_CONFIG0_ODR_200HZ   0x07
#define B_IIM42653_GYRO_CONFIG0_ODR_500HZ   0x0F
#define B_IIM42653_GYRO_CONFIG0_FSR_4000DPS 0x00
#define B_IIM42653_GYRO_CONFIG0_FSR_250DPS  0x04

// ACCEL_CONFIG0 [Configuration, Sensor FSR/ODR]
#define R_IIM42653_ACCEL_CONFIG0 0x50
typedef union {
    struct {
        uint8_t ACCEL_ODR :4;		//ODR, see datasheet for range
        bool RESERVED :1;			//Reserved
        uint8_t ACCEL_UI_FS_SEL :3;	//FSR, see datasheet for range
    } fields;
    uint8_t data;
} IIM42653_ACCEL_CONFIG0;
#define B_IIM42653_ACCEL_CONFIG0_ODR_100HZ 0x08
#define B_IIM42653_ACCEL_CONFIG0_ODR_200HZ 0x07
#define B_IIM42653_ACCEL_CONFIG0_ODR_500HZ 0x0F
#define B_IIM42653_ACCEL_CONFIG0_FSR_32G   0x00
#define B_IIM42653_ACCEL_CONFIG_FSR_4G     0x03

// WHO_AM_I [Validation] 
#define R_IIM42653_WHO_AM_I 0x75
#define B_IIM42653_WHO_AM_I_VALUE 0x56

class IIM42653 {
	public: 
        IIM42653(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

        void sample();

        int16_t get_ax() { return ax; }
        int16_t get_ay() { return ay; }
        int16_t get_az() { return az; }
        int16_t get_gx() { return gx; }
        int16_t get_gy() { return gy; }
        int16_t get_gz() { return gz; }

        static float scale_accel(int16_t unscaled) { return ((float) unscaled) / S_IIM42653_ACCEL_SENSITIVITY_FACTOR; }

        static float scale_gyro(int16_t unscaled) { return ((float) unscaled) / S_IIM42653_GYRO_SENSITIVITY_FACTOR; }

	private:
        const uint8_t addr = IIM42653_I2C_ADDR;

        i2c_inst_t* i2c;

        uint8_t buffer[16];

        IIM42653_DEVICE_CONFIG device_config;
        IIM42653_DRIVE_CONFIG drive_config;
        IIM42653_PWR_MGMT0 pwr_mgmt0;
        IIM42653_GYRO_CONFIG0 gyro_config0;
        IIM42653_ACCEL_CONFIG0 accel_config0;

        //Internal data fields
        int16_t ax, ay, az, gx, gy, gz;
};
