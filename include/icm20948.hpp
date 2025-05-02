#pragma once

#include "serial.hpp"
#include <stdint.h>

#include <hardware/i2c.h>

#if (USE_FREERTOS == 1)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"
#endif

#define ICM20948_I2C_ADDR 0x69

// WHO_AM_I [Validation]
#define R_ICM20948_B0_WHO_AM_I 0x00
#define B_ICM20948_WHO_AM_I_VALUE 0xEA

// USER_CTRL [Configuration, mainly auxilary I2C bus enabling]
#define R_ICM20948_B0_USER_CTRL 0x01

// Data output registers
#define R_ICM20948_B0_ACCEL_XOUT_H 0x2D

// REG_BANK_SEL [Configuration, register bank]
#define R_ICM20948_REG_BANK_SEL 0x7F

// I2C_MST_CTRL [Configuration, auxilary I2C bus speed and enable]
#define R_ICM20948_B3_I2C_MST_CTRL 0x01

// I2C_MST_STATUS [Status, check ongoing auxilary I2C operations]
#define R_ICM20948_B0_I2C_MST_STATUS 0x17
#define B_ICM20948_I2C_SLV4_DONE_MASK 0x40

// I2C_SLV4_ADDR [Configuration, auxilary I2C bus operations (Address of slave)]
#define R_ICM20948_B3_I2C_SLV4_ADDR 0x13

// I2C_SLV4_REG [Configuration, auxilary I2C bus operations (Register of slave)]
#define R_ICM20948_B3_I2C_SLV4_REG 0x14

// I2C_SLV4_CTRL [Configuration, auxilary I2C bus operations (Control of I2C operations)]
#define R_ICM20948_B3_I2C_SLV4_CTRL 0x15
typedef union {
    struct {
        uint8_t I2C_SLV4_DLY :5;    //Slave enable oversample divider???
        bool I2C_SLV4_REG_DIS :1;   //Disable register writes?? Datasheet is vague (and stupid and dumb and stupid)
        bool I2C_SLV4_INT_EN :1;    //Enable interrupts on auxilary I2C op completion
        bool I2C_SLV4_EN :1;        //Enable auxilary I2C operations with this slave
    } fields;
    uint8_t data;
} ICM20948_I2C_SLV4_CTRL;

// I2C_SLV4_DO ["Data out when slave 4 is set to write"]
#define R_ICM20948_B3_I2C_SLV4_DO 0x16

// I2C_SLV4_DI ["Data read from i2c slave 4"]
#define R_ICM20948_B3_I2C_SLV4_DI 0x17

#define ICM20948_SAMPLE_RATE_HZ 500

class ICM20948 {
    public:
        ICM20948(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

        void sample();
        
        int16_t get_ax() { return ax; }
        int16_t get_ay() { return ay; }
        int16_t get_az() { return az; }
        int16_t get_gx() { return gx; }
        int16_t get_gy() { return gy; }
        int16_t get_gz() { return gz; }
        int16_t get_mx() { return mx; }
        int16_t get_my() { return my; }
        int16_t get_mz() { return mz; }
        int16_t get_temp() { return temp; }

#if ( USE_FREERTOS == 1 )
        static void update_icm20948_task(void* pvParameters);

        TaskHandle_t update_task_handle = NULL;
#endif

    private:
        //Helper method for switching user register bank
        void set_register_bank(uint8_t bank);

        //Helper methods for dealing with the stupid-ass way that this chip handles magnetometer communication
        void write_aux_register(uint8_t slv_addr, uint8_t slv_reg, uint8_t value);
        uint8_t read_aux_register(uint8_t slv_addr, uint8_t slv_reg);
        void configure_mag_i2c();

        //Auxilary (internal) I2C handling and register fields
        ICM20948_I2C_SLV4_CTRL slv4_ctrl;

        //External i2c handling and register fields
        const uint8_t addr = ICM20948_I2C_ADDR;
        i2c_inst_t* i2c;
        uint8_t buffer[24];

        //Internal data fields
        int16_t raw_ax, raw_ay, raw_az;
        int16_t raw_gx, raw_gy, raw_gz;
        int16_t raw_mx, raw_my, raw_mz;
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz, temp;
};
