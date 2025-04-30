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
#define R_ICM20948_WHO_AM_I 0x00
#define B_ICM20948_WHO_AM_I_VALUE 0xEA

class ICM20948 {
    public:
        ICM20948(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

    private:
        const uint8_t addr = ICM20948_I2C_ADDR;

        i2c_inst_t* i2c;

        uint8_t buffer[16];
};
