#pragma once

#include <stdint.h>

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "pico/time.h"
#include "sensor_i2c.hpp"

#if ( USE_FREERTOS == 1 )
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"
#endif

#define MS5607_I2C_ADDRESS 0x77

#define PROM_MANUFACTURER_RESERVED_ADDR    0x0
#define PROM_CALIBRATION_COEFFICENT_1_ADDR 0x1
#define PROM_CALIBRATION_COEFFICENT_2_ADDR 0x2
#define PROM_CALIBRATION_COEFFICENT_3_ADDR 0x3
#define PROM_CALIBRATION_COEFFICENT_4_ADDR 0x4
#define PROM_CALIBRATION_COEFFICENT_5_ADDR 0x5
#define PROM_CALIBRATION_COEFFICENT_6_ADDR 0x6
#define PROM_CRC_ADDR                      0x7

#define OSR_CONVERT_256 0x0
#define OSR_CONVERT_512 0x1
#define OSR_CONVERT_1024 0x2
#define OSR_CONVERT_2048 0x3
#define OSR_CONVERT_4096 0x4

#define OSR_256_CONVERSION_TIME_US 600

#define TYPE_UNCOMPENSATED_PRESSURE 0
#define TYPE_UNCOMPENSATED_TEMPERATURE 1

#define RESET_COMMAND 0x1E

#define ADC_READ_COMMAND 0x0

#define ALTITUDE_SCALE_F 10.0f
#define PRESSURE_SCALE_F 100.0f
#define TEMPERATURE_SCALE_F 100.0f

#define ALTITUDE_SCALE 10
#define PRESSURE_SCALE 100
#define TEMPERATURE_SCALE 100

typedef union {
	struct {
        uint8_t RESERVED: 1;
        uint8_t ADDR_OSR: 3;
        uint8_t TYPE :1;
		bool PROM2 :1;
		bool CONVERT :1;
        bool PROM: 1;
	} fields;
	uint8_t data;
} ms5607_cmd;

typedef enum {
    NOT_SAMPLING,
    PRESSURE_CONVERT,
    TEMPERATURE_CONVERT,
    COMPENSATE
} sample_state_t;

class altimeter {
    public:
        altimeter(i2c_inst_t* i2c) : i2c {i2c} {};

        void initialize();

        void ms5607_write_cmd(ms5607_cmd* cmd);

        void ms5607_start_sample();

        void ms5607_sample();

        static int64_t ms5607_sample_callback(alarm_id_t id, void* user_data);
#if (USE_FREERTOS == 1)
        static void ms5607_sample_handler(void* pvParameters);
#endif

        int32_t pressure_to_altitude(int32_t pressure);

        int32_t get_pressure() { return pressure; }
        int32_t get_temperature() { return temperature; }
        int32_t get_altitude() { return altitude; }

        void set_threshold_altitude(int32_t threshold_altitude, alarm_callback_t callback);

        void clear_threshold_altitude();

#if ( USE_FREERTOS == 1 )
        TaskHandle_t sample_handler_task = NULL;
#endif
    private:
        void ms5607_compensate();

        uint8_t buffer[3];

        uint16_t prom[6];

        uint32_t uncompensated_pressure;
        uint32_t uncompensated_temperature;

        int32_t pressure;

        int32_t temperature;

        int32_t altitude;

        sample_state_t sample_state;

        int32_t threshold_altitude;

        alarm_callback_t threshold_callback  = NULL;

        bool positive_crossing;

        i2c_inst_t* i2c;
        const uint8_t addr = MS5607_I2C_ADDRESS;
};
