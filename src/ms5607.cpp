#include "ms5607.hpp"

static const int32_t altitude_table[] = {
#include "altitude-pa.h"
};

#define ALT_SCALE (1 << ALT_SHIFT)
#define ALT_MASK (ALT_SCALE - 1)

void MS5607::initialize() {
    sample_state = NOT_SAMPLING;

    alarm_pool_init_default();

    ms5607_cmd cmd;
    cmd.data = RESET_COMMAND;

    ms5607_write_cmd(&cmd);

    sleep_ms(500);

    cmd.data = 0;
    cmd.fields.PROM = true;
    cmd.fields.PROM2 = true;

    cmd.fields.ADDR_OSR = PROM_CALIBRATION_COEFFICENT_1_ADDR;

    for (uint8_t prom_addr = PROM_CALIBRATION_COEFFICENT_1_ADDR; prom_addr <= PROM_CALIBRATION_COEFFICENT_6_ADDR; prom_addr++) {
        sleep_ms(100);
        cmd.fields.ADDR_OSR = prom_addr;
        ms5607_write_cmd(&cmd);
        i2c_read_blocking(i2c, addr, buffer, 2, false);

        prom[prom_addr - 1] = static_cast<uint16_t>((((uint16_t) buffer[0]) << 8) | ((uint16_t) buffer[1]));
    }
}

void MS5607::ms5607_write_cmd(ms5607_cmd* cmd) {
    i2c_write_blocking(i2c, addr, (uint8_t *) cmd, 1, false);
}

void MS5607::ms5607_start_sample() {
    if (sample_state == NOT_SAMPLING) {
        sample_state = PRESSURE_CONVERT;
        ms5607_sample();
    }
}

#if (USE_FREERTOS == 1)
void MS5607::ms5607_sample_handler(void* pvParameters) {
    /* xMaxExpectedBlockTime is set to be a little longer than the maximum
    expected time between events. */
    const TickType_t xInterruptFrequency = pdMS_TO_TICKS( 1000 / (MS5607_SAMPLE_RATE_HZ * 2) );
    const TickType_t xMaxExpectedBlockTime = xInterruptFrequency + pdMS_TO_TICKS( 1 );
    uint32_t ulEventsToProcess;
    while (1) {
        /* Wait to receive a notification sent directly to this task from the
        interrupt service routine. */
        ulEventsToProcess = ulTaskNotifyTake( pdTRUE, xMaxExpectedBlockTime );
        if( ulEventsToProcess != 0 ) {
            /* To get here at least one event must have occurred. Loop here
            until all the pending events have been processed */
            while( ulEventsToProcess > 0 ) {
                MS5607* alt = (MS5607 *) pvParameters;
                alt->ms5607_sample();
                ulEventsToProcess--;
            }
        }
    }

}

void MS5607::update_ms5607_task(void* pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / MS5607_SAMPLE_RATE_HZ);

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        MS5607* alt = (MS5607 *) pvParameters;
        alt->ms5607_start_sample();
    }
}
#endif

int64_t MS5607::ms5607_sample_callback(alarm_id_t id, void* user_data) {
#if ( USE_FREERTOS == 1 )
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    MS5607* alt = (MS5607 *) user_data;
    // Defer ISR handling to separate handler within FreeRTOS context
    vTaskNotifyGiveFromISR(alt->sample_handler_task_handle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
#else
    MS5607* alt = (MS5607 *) user_data;
    alt->ms5607_sample();
#endif
    return 0;
}

void MS5607::ms5607_sample() {
    ms5607_cmd cmd = {.data = 0};

    switch (sample_state) {
        case NOT_SAMPLING:
            break;
        case PRESSURE_CONVERT: {
            cmd.fields.CONVERT = 1;
            cmd.fields.TYPE = TYPE_UNCOMPENSATED_PRESSURE;
            cmd.fields.ADDR_OSR = OSR_CONVERT_256;

            ms5607_write_cmd(&cmd);

            add_alarm_in_us(OSR_256_CONVERSION_TIME_US, MS5607::ms5607_sample_callback, (void *) this, true);

            sample_state = TEMPERATURE_CONVERT;
            break;
        };
        case TEMPERATURE_CONVERT: {
            cmd.data = ADC_READ_COMMAND;
            ms5607_write_cmd(&cmd);
            i2c_read_blocking(i2c, addr, buffer, 3, false);
            uncompensated_pressure = (((uint32_t) buffer[0]) << 16) | (((uint32_t) buffer[1]) << 8) | ((uint32_t) buffer[2]);

            cmd.fields.CONVERT = 1;
            cmd.fields.TYPE = TYPE_UNCOMPENSATED_TEMPERATURE;
            cmd.fields.ADDR_OSR = OSR_CONVERT_256;

            ms5607_write_cmd(&cmd);

            add_alarm_in_us(OSR_256_CONVERSION_TIME_US, MS5607::ms5607_sample_callback, (void *) this, true);

            sample_state = COMPENSATE;
            break;
        };
        case COMPENSATE: {
            cmd.data = ADC_READ_COMMAND;
            ms5607_write_cmd(&cmd);
            i2c_read_blocking(i2c, addr, buffer, 3, false);
            uncompensated_temperature = (((uint32_t) buffer[0]) << 16) | (((uint32_t) buffer[1]) << 8) | ((uint32_t) buffer[2]);
            ms5607_compensate();
            altitude = pressure_to_altitude(pressure);


            sample_state = NOT_SAMPLING;
            if (threshold_callback != NULL) {
                if (positive_crossing) {
                    if (altitude >= threshold_altitude) {
                        add_alarm_in_ms(1, threshold_callback, NULL, true);
                    }
                } else {
                    if (altitude <= threshold_altitude) {
                        add_alarm_in_ms(1, threshold_callback, NULL, true);
                    }
                }
            }
            break;
        };
    };

}

void MS5607::ms5607_compensate() {
    int32_t dT = uncompensated_temperature - (((uint32_t) prom[4]) << 8);
    temperature = 2000 + ( ( ( (int64_t) dT) * ( (int64_t) prom[5]) ) >> 23);
    int64_t OFF = ( ( (int64_t) prom[1]) << 17) + ( ( ((int64_t) prom[3]) * ( (int64_t) dT)) >> 6);
    int64_t SENS = ( ( (int64_t) prom[0]) << 16) + (( ( (int64_t) prom[2]) * ((int64_t) dT)) >> 7);
    pressure = (int32_t) ((((((int64_t) uncompensated_pressure) * SENS) >> 21) - OFF) >> 15);

}

int32_t MS5607::pressure_to_altitude(int32_t pressure) {
	uint16_t o;
	int16_t	part;
	int32_t low, high;

	if (pressure < 0) {
		pressure = 0;
    }

	if (pressure > 120000) {
		pressure = 120000;
    }

	o = (uint16_t) (pressure >> ALT_SHIFT);
	part = pressure & ALT_MASK;

	low = (int32_t) altitude_table[o] * (ALT_SCALE - part);
	high = (int32_t) altitude_table[o + 1] * part;
	return ((low + high + (ALT_SCALE >> 1)) >> ALT_SHIFT);
}


void MS5607::set_threshold_altitude(int32_t threshold_altitude, alarm_callback_t callback) {
    this->threshold_altitude = threshold_altitude;

    positive_crossing = (threshold_altitude > altitude);

    threshold_callback = callback;
}

void MS5607::clear_threshold_altitude() {
    this->threshold_callback = NULL;
}
