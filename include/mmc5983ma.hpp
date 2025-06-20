#pragma once

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "serial.hpp"

#if ( DEBUG == 1 )
#include <stdio.h>
#include <inttypes.h>
#include "pico/stdio.h"
#endif

#if ( USE_FREERTOS == 1)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "serial.hpp"
#include "task.h"
#include "semphr.h"
#endif

#define MMC5983MA_I2C_ADDR 0x30

#define R_MMC5983MA_XOUT0 0x00
#define R_MMC5983MA_XOUT1 0x01
#define R_MMC5983MA_YOUT0 0x02
#define R_MMC5983MA_YOUT1 0x03
#define R_MMC5983MA_ZOUT0 0x04
#define R_MMC5983MA_ZOUT1 0x05
#define R_MMC5983MA_XYZOUT2 0x06
#define R_MMC5983MA_TEMPERATURE_OUT 0x07
#define S_MMC5983MA_SCALE_FACTOR_16BIT 4096
#define S_MMC5983MA_SCALE_FACTOR_18BIT 16384

#define R_MMC5983MA_DEV_STATUS 0x08
typedef union {
    struct {
        bool MEAS_M_DONE :1;
        bool MEAS_T_DONE :1;
        uint8_t RESERVED0 :2;
        bool OTP_READ_DONE :1;
        uint8_t RESERVED1 :3;
    } fields;
    uint8_t data;
} MMC5983MA_DEV_STATUS;

#define R_MMC5983MA_INTERNAL_CTL0 0x09
typedef union {
    struct {
        bool TAKE_MAG_MEAS :1;
        bool TAKE_TEMP_MEAS :1;
        bool MEAS_INT_ENABLE :1;
        bool SET_CMD :1;
        bool RESET_CMD :1;
        bool AUTO_SR_ENABLE :1;
        bool OTP_READ :1;
        uint8_t RESERVED :1;
    } fields;
    uint8_t data;
} MMC5983MA_INTERNAL_CTL0;

#define R_MMC5983MA_INTERNAL_CTL1 0x0A
typedef union {
    struct {
        uint8_t BANDWIDTH: 2;
        bool X_INHIBIT: 1;
        bool Y_INHIBIT: 1;
        bool Z_INHIBIT: 1;
        uint8_t RESERVED :2;
        bool RESTART :1;
    } fields;
    uint8_t data;
} MMC5983MA_INTERNAL_CTL1;
#define B_MMC5983MA_BANDWIDTH_100HZ 0b00
#define B_MMC5983MA_BANDWIDTH_200HZ 0b01
#define B_MMC5983MA_BANDWIDTH_400HZ 0b10
#define B_MMC5983MA_BANDWIDTH_800HZ 0b11

#define R_MMC5983MA_INTERNAL_CTL2 0x0B
typedef union {
    struct {
        bool CONTINUOUS_MODE_FREQ :3;
        bool CONTINUOUS_MODE_ENABLE :1;
        uint8_t PERIODIC_SET_RATE :3;
        bool PERIODIC_SET_ENABLE :1;
    } fields;
    uint8_t data;
} MMC5983MA_INTERNAL_CTL2;
#define B_MMC5983_CONTINUOUS_MODE_OFF           0b000
#define B_MMC5983_CONTINUOUS_MODE_FREQ_1HZ      0b001
#define B_MMC5983_CONTINUOUS_MODE_FREQ_10HZ     0b010
#define B_MMC5983_CONTINUOUS_MODE_FREQ_20HZ     0b011
#define B_MMC5983_CONTINUOUS_MODE_FREQ_50HZ     0b100
#define B_MMC5983_CONTINUOUS_MODE_FREQ_100HZ    0b101
#define B_MMC5983_CONTINUOUS_MODE_FREQ_200HZ    0b110
#define B_MMC5983_CONTINUOUS_MODE_FREQ_1000HZ   0b111

#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_1    0b000
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_25   0b001
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_75   0b010
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_100  0b011
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_250  0b100
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_500  0b101
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_1000 0b110
#define B_MMC5983_PERIODIC_SET_RATE_MEAS_TIMES_2000 0b111

#define R_MMC5983MA_INTERNAL_CTL3 0x0C
typedef union {
    struct {
        uint8_t RESERVED0 :1;
        bool ST_ENABLE_POS_NEG :1;
        bool ST_ENABLE_NEG_POS :1;
        uint8_t RESERVED1 :3;
        bool SPI_3_WIRE_ENABLE :1;
        uint8_t RESERVED2 :1;
    } fields;
    uint8_t data;
} MMC5983MA_INTERNAL_CTL3;

#define R_MMC5983MA_PRODUCT_ID 0x2F
#define B_MMC5983MA_PRODUCT_ID 0x30

#define MMC5983_SAMPLE_RATE_HZ 200

class MMC5983MA {
    public:
        MMC5983MA(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

        void calibrate();

        void sample();

        void apply_offset();

        int16_t get_ax() { return ax; }
        int16_t get_ay() { return ay; }
        int16_t get_az() { return az; }

        static float scale_mag(int16_t unscaled) { return ((float) unscaled) / S_MMC5983MA_SCALE_FACTOR_16BIT; }

#if (USE_FREERTOS == 1)
        static void update_mmc5983ma_task(void* pvParameters);

        TaskHandle_t update_task_handle = NULL;
#endif

    private:
        static int16_t sat_sub(int16_t a, int16_t b);

        const uint8_t addr = MMC5983MA_I2C_ADDR;

        i2c_inst_t* i2c;

        uint8_t buffer[16];

        MMC5983MA_DEV_STATUS dev_status;
        MMC5983MA_INTERNAL_CTL0 internal_ctl0;
        MMC5983MA_INTERNAL_CTL1 internal_ctl1;
        MMC5983MA_INTERNAL_CTL2 internal_ctl2;
        MMC5983MA_INTERNAL_CTL3 internal_ctl3;

        int16_t offset_x = 0, offset_y = 0, offset_z = 0;
        int16_t raw_x = 0, raw_y = 0, raw_z = 0;
        int16_t ax = 0, ay = 0, az = 0;
};
