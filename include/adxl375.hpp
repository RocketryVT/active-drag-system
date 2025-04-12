#pragma once

#include <stdint.h>

#include <hardware/i2c.h>

#define ADXL375_I2C_ADDRESS 0x1D

// DEVID [Validation]
#define R_ADXL375_DEVID 		0x00
#define B_ADXL375_DEVID_VALUE 	0xE5

// THRESH_SHOCK [Configuration, Threshold for shock interrupt (780mG/LSB)]
#define R_ADXL375_THRESH_SHOCK 			0x1D
typedef uint8_t ADXL375_THRESH_SHOCK;

// OFSX, OFSY, OFSZ [Configuration, User-configured offset of (0.196 g/LSB)]
#define R_ADXL375_OFFSET_X 0x1E
typedef uint8_t ADXL375_OFFSET_X;

#define R_ADXL375_OFFSET_Y 0x1F
typedef uint8_t ADXL375_OFFSET_Y;

#define R_ADXL375_OFFSET_Z 0x20
typedef uint8_t ADXL375_OFFSET_Z;

// DUR [Configuration, Time above threshold as single shock interrupt (625us/LSB)]
#define R_ADXL375_DUR 0x21
typedef uint8_t ADXL375_DURATION;

// LATENT [Configuration, Time after first shock interrupt second can occur (1.25ms/LSB)]
#define R_ADXL375_LATENT 0x22
typedef uint8_t ADXL375_LATENT;

// WINDOW [Configuration, Time after latent period second shock can begin (1.25ms/LSB)]
#define R_ADXL375_WINDOW 0x23
typedef uint8_t ADXL375_WINDOW;

// THRESH_ACT [Configuration, Threshold value for detecting activity (780mg/LSB)]
#define R_ADXL375_THRESH_ACT 0x24
typedef uint8_t ADXL375_THRESH_ACT;

// THRESH_INACT [Configuration, Threshold value for detecting inactivity (780mg/LSB)]
#define R_ADXL375_THRESH_INACT 0x25
typedef uint8_t ADXL375_THRESH_INACT;

// TIME_INACT [Configuration, Time below threshold as inactivity (1s/LSB)]
#define R_ADXL375_TIME_INACT 0x26
typedef uint8_t ADXL375_TIME_INACT;

// ACT_INACT_CTL [Configuration, Activity/Inactivity interrupts enable and AC/DC coupling]
#define R_ADXL375_ACT_INACT_CTL 0x27
typedef union {
    struct {
        bool INACT_Z_ENABLE :1;
        bool INACT_Y_ENABLE :1;
        bool INACT_X_ENABLE :1;
        bool INACT_AC_DC :1;
        bool ACT_Z_ENABLE :1;
        bool ACT_Y_ENABLE :1;
        bool ACT_X_ENABLE :1;
        bool ACT_AC_DC :1;
    } fields;
    uint8_t data;
} ADXL375_ACT_INACT_CTL;

// SHOCK_AXES [Configuration, axes used in shock interrupt]
#define R_ADXL375_SHOCK_AXES 0x2A
typedef union {
    struct {
        bool SHOCK_Z_ENABLE :1; //Z-Axis shock interrupt enable
        bool SHOCK_Y_ENABLE :1; //Y-Axis shock interrupt enable
        bool SHOCK_X_ENABLE :1; //X-Axis shock interrupt enable
        bool SUPPRESS :1;       //Suppresses double shock detection
        uint8_t RESERVED :4;    //Reserved
    } fields;
    uint8_t data;	
} ADXL375_SHOCK_AXES;

// ACT_SHOCK_STATUS [Status, First axis involved in an activity or shock event]
#define R_ADXL375_ACT_SHOCK_STATUS 0x2B
typedef union {
    struct {
        bool SHOCK_Z_SOURCE :1;
        bool SHOCK_Y_SOURCE :1;
        bool SHOCK_X_SOURCE :1;
        bool ASLEEP :1;
        bool ACT_Z_SOURCE :1;
        bool ACT_Y_SOURCE :1;
        bool ACT_X_SOURCE :1;
        bool RESERVED :1;
    } fields;
    uint8_t data;
} ADXL375_ACT_SHOCK_STATUS;

// BW_RATE [Configuration, Low Power Mode and ODR]
#define R_ADXL375_BW_RATE 0x2C
typedef union {
    struct {
        uint8_t RATE :4;        //ODR configuration
        bool LOW_POWER :1;      //Low-power/Normal mode configuration
        uint8_t RESERVED :3;    //Reserved
    } fields;
    uint8_t data;
} ADXL375_BW_RATE;
#define B_ADXL375_ODR_100_HZ                0x0A
#define B_ADXL375_ODR_400_HZ                0x0C
#define B_ADXL375_ODR_800_HZ                0x0D
#define B_ADXL375_ODR_3200_HZ               0x0F
#define B_ADXL375_BW_RATE_LOW_POWER_MODE    0b1
#define B_ADXL375_BW_RATE_NORMAL_POWER_MODE 0b0
		
// POWER_CTL [State Configuration, Measurement mode (and sleep settings)]
#define R_ADXL375_POWER_CTL 0x2D
typedef union {
    struct {
        uint8_t WAKEUP :2;  //Sampling rate control during sleep mode
        bool SLEEP :1;      //Sleep mode configuration
        bool MEASURE :1;    //Measurement/Standby mode configuration
        bool AUTO_SLEEP :1; //Auto sleep function enable
        bool LINK :1;       //Link activity and inactivity functions
        bool RESERVED :1;   //Reserved
    } fields;
    uint8_t data;
} ADXL375_POWER_CTL;

// INT_ENABLE [State Configuration, interrupt enabling]
#define R_ADXL375_INT_ENABLE 0x2E
typedef union {
    struct {
        bool OVERRUN :1;        //FIFO buffer overwrite interrupt enable
        bool WATERMARK :1;      //??? TODO: Figure out watermark bits
        bool RESERVED :1;       //Reserved
        bool INACTIVITY :1;     //Inactivity interrupt enable
        bool ACTIVITY :1;       //Activity interrupt enable
        bool DOUBLE_SHOCK :1;   //Double shock interrupt enable
        bool SINGLE_SHOCK :1;   //Single shock interrupt enable
        bool DATA_READY :1;     //FIFO buffer new data interrupt enable
    } fields;
    uint8_t data;
} ADXL375_INT_ENABLE;

// INT_MAP [Configuration, Interrupt --> Pin mapping (0 = INT1, 1 = INT2) */
#define R_ADXL375_INT_MAP 0x2F
typedef union {
	struct {
		bool OVERRUN :1;		//FIFO buffer overwrite interrupt
		bool WATERMARK :1;		//??? TODO: Figure out watermark bits
		bool RESERVED :1;		//Reserved
		bool INACTIVITY :1;		//Inactivity interrupt
		bool ACTIVITY :1;		//Activity interrupt
		bool DOUBLE_SHOCK :1;	//Double shock interrupt
		bool SINGLE_SHOCK :1;	//Single shock interrupt
		bool DATA_READY :1;		//FIFO buffer new data interrupt
	} fields;
	uint8_t data;
} ADXL375_INT_MAP;

// INT_SOURCE [Status, Source of interrupt (read to clear SINGLE_SHOCK interrupt) */
#define R_ADXL375_INT_SOURCE 0x30
typedef union {
    struct {
        bool OVERRUN :1;        //FIFO buffer overwrite interrupt
        bool WATERMARK :1;      //??? TODO: Figure out watermark bits
        bool RESERVED :1;       //Reserved
        bool INACTIVITY :1;     //Inactivity interrupt
        bool ACTIVITY :1;       //Activity interrupt
        bool DOUBLE_SHOCK :1;   //Double shock interrupt
        bool SINGLE_SHOCK :1;   //Single shock interrupt
        bool DATA_READY :1;     //FIFO buffer new data interrupt
    } fields;
    uint8_t data;
} ADXL375_INT_SOURCE;

// DATA_FORMAT [Configuration, Data output style/formatting (variety of things)] */
#define R_ADXL375_DATA_FORMAT 0x31
typedef union {
    struct {
    uint8_t RESERVED :2;    //Reserved
    bool JUSTIFY :1;        //Left(MSB)/Right(LSB, Sign extended) output justification
    uint8_t RESERVED_ :2;   //Reserved
    bool INT_INVERT :1;     //Interrupt pin polarity configuration
    bool SPI :1;            //SPI 3/4 wire mode configuration
    bool SELF_TEST :1;      //Configure self-test mode
    } fields;
    uint8_t data;
} ADXL375_DATA_FORMAT;
#define B_ADXL375_DATA_FORMAT_JUSTIFY_RIGHT         0b0
#define B_ADXL375_DATA_FORMAT_JUSTIFY_LEFT          0b1
#define B_ADXL375_DATA_FORMAT_INTERRUPT_ACTIVE_HIGH 0b0
#define B_ADXL375_DATA_FORMAT_INTERRUPT_ACTIVE_LOW  0b1
#define B_ADXL375_DATA_FORMAT_ENABLE_SELF_TEST      0b1
#define B_ADXL375_DATA_FORMAT_DISABLE_SELF_TEST     0b0
		
// DATAX0-DATAZ1 [Output, 16b data for X/Y/Z (DATA_1 is MSByte, DATA_0 is LSByte)
#define R_ADXL375_DATAX0 0x32
#define R_ADXL375_DATAX1 0x33
#define R_ADXL375_DATAY0 0x34
#define R_ADXL375_DATAY1 0x35
#define R_ADXL375_DATAZ0 0x36
#define R_ADXL375_DATAZ1 0x37
#define S_ADXL375_SCALE_FACTOR 20.5 // 1 LSB / 20.5

class ADXL375 {
    public: 
        ADXL375(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

        void sample();

        int16_t get_ax() { return ax; }
        int16_t get_ay() { return ay; }
        int16_t get_az() { return az; }

        static float scale(int16_t unscaled) { return ((float) unscaled) / S_ADXL375_SCALE_FACTOR; }

    private:
        const uint8_t addr = ADXL375_I2C_ADDRESS;

        i2c_inst_t* i2c;

        uint8_t buffer[16];

        ADXL375_THRESH_SHOCK shock_threshold;
        ADXL375_OFFSET_X offset_ax;
        ADXL375_OFFSET_Y offset_ay;
        ADXL375_OFFSET_Z offset_az;
        ADXL375_DURATION shock_duration;
        ADXL375_LATENT latent_shock;
        ADXL375_WINDOW window_shock;
        ADXL375_THRESH_ACT activity_threshold;
        ADXL375_THRESH_INACT inactivity_threshold;
        ADXL375_TIME_INACT inactivity_duraction;
        ADXL375_ACT_INACT_CTL act_inact_ctl;
        ADXL375_SHOCK_AXES shock_axes;
        ADXL375_ACT_SHOCK_STATUS act_shock_status;
        ADXL375_BW_RATE bw_rate;
        ADXL375_POWER_CTL power_ctl;
        ADXL375_INT_ENABLE int_enable;
        ADXL375_INT_MAP	int_map;
        ADXL375_INT_SOURCE int_source;
        ADXL375_DATA_FORMAT data_format;

        int16_t ax, ay, az;
};

