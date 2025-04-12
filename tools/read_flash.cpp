#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include <hardware/timer.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include <pico/error.h>
#include <pico/types.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "serial.hpp"
#include "task.h"
#include "semphr.h"

#include "adxl375.hpp"
#include "ms5607.hpp"
#include "iim42653.hpp"
#include "serial.hpp"

/* Priorities at which the tasks are created. */
#define EVENT_HANDLER_PRIORITY      ( tskIDLE_PRIORITY + 4 )
#define SENSOR_SAMPLE_PRIORITY      ( tskIDLE_PRIORITY + 3 )
#define	HEARTBEAT_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define LOGGING_PRIORITY            ( tskIDLE_PRIORITY + 2 )
#define	SERIAL_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

#define HEARTBEAT_RATE_HZ 5
#define DATA_UPDATE_RATE_HZ 100

#define MAX_SCL 400000

static void sample_cmd_func();
static void debug_cmd_func();
static void logging_task(void * unused_arg);
static void update_data_task( void *pvParameters );
static void heartbeat_task( void *pvParameters );

static void update_ms5607_task( void *pvParameters );

static void update_adxl375_task( void *pvParameters );
static void update_iim42653_task( void *pvParameters );

void vApplicationTickHook(void) { /* optional */ }
void vApplicationMallocFailedHook(void) { /* optional */ }
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) { for( ;; ); }

const char* executeable_name = "read_flash.uf2";
const size_t num_user_cmds = 2;
const command_t user_commands[] = { {.name = "sample",
                                     .len = 6,
                                     .function = &sample_cmd_func},
                                    {.name = "debug",
                                     .len = 5,
                                     .function = &debug_cmd_func} };

volatile bool serial_data_output = false;

altimeter alt(i2c_default);
ADXL375 adxl375(i2c_default);
IIM42653 iim42653(i2c_default);

int main() {
    stdio_init_all();

    adc_init();
    adc_set_temp_sensor_enabled(true);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(2500);

    info_cmd_func();
    stdio_flush();

    xTaskCreate(heartbeat_task, "heartbeat", 256, NULL, HEARTBEAT_TASK_PRIORITY, NULL);
    xTaskCreate(serial_task, "serial", 8192, NULL, SERIAL_TASK_PRIORITY, NULL);

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
}

static void update_ms5607_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 500);

    alt.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        alt.ms5607_start_sample();
    }
}

static void update_adxl375_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 500);

    adxl375.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        taskENTER_CRITICAL();
        adxl375.sample();
        taskEXIT_CRITICAL();
        if ((xLastWakeTime + xFrequency) < xTaskGetTickCount()) {
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}

static void update_iim42653_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 500);

    iim42653.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        taskENTER_CRITICAL();
        iim42653.sample();
        taskEXIT_CRITICAL();
        if ((xLastWakeTime + xFrequency) < xTaskGetTickCount()) {
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}

static void logging_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 10);

    xLastWakeTime = xTaskGetTickCount();
    printf("Time,Pressure,Altitude,Temperature,ax,ay,az,ax,ay,az,gx,gy,gz\n");
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        printf("%" PRIu64 ",", time_us_64());
        printf("%4.2f,", ((float) alt.get_pressure()) / PRESSURE_SCALE_F);
        printf("%4.2f,", ((float) alt.get_altitude()) / ALTITUDE_SCALE_F);
        printf("%4.2f,", ((float) alt.get_temperature()) / TEMPERATURE_SCALE_F);
        printf("%4.2f,", adxl375.scale(adxl375.get_ax()));
        printf("%4.2f,", adxl375.scale(adxl375.get_ay()));
        printf("%4.2f,", adxl375.scale(adxl375.get_az()));
        printf("%4.2f,", iim42653.scale_accel(iim42653.get_ax()));
        printf("%4.2f,", iim42653.scale_accel(iim42653.get_ay()));
        printf("%4.2f,", iim42653.scale_accel(iim42653.get_az()));
        printf("%4.2f,", iim42653.scale_gyro(iim42653.get_gx()));
        printf("%4.2f,", iim42653.scale_gyro(iim42653.get_gy()));
        printf("%4.2f", iim42653.scale_gyro(iim42653.get_gz()));
        printf("\r\n");
        stdio_flush();
    }

}

static void heartbeat_task( void *pvParameters ) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / HEARTBEAT_RATE_HZ);
    static volatile uint8_t led_counter = 0;
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        bool led_status = sequence[led_counter];
        gpio_put(PICO_DEFAULT_LED_PIN, led_status);
        led_counter++;
        led_counter %= sequence_length;
    }
}

static void sample_cmd_func() {
    static TaskHandle_t ms5607_handle   = NULL;
    static TaskHandle_t adxl375_handle  = NULL;
    static TaskHandle_t iim42653_handle = NULL;
    static TaskHandle_t mmc5883_handle  = NULL;
    static bool sampling = false;

    if (sampling == false) {
        vTaskSuspendAll();

        xTaskCreate(update_ms5607_task, "update_ms5607", 256, NULL, SENSOR_SAMPLE_PRIORITY, &ms5607_handle);
        xTaskCreate(update_adxl375_task, "update_adxl375", 256, NULL, SENSOR_SAMPLE_PRIORITY, &adxl375_handle);
        xTaskCreate(update_iim42653_task, "update_iim42653", 256, NULL, SENSOR_SAMPLE_PRIORITY, &iim42653_handle);

        xTaskCreate(altimeter::ms5607_sample_handler, "ms5607_sample_handler", 256, &alt, EVENT_HANDLER_PRIORITY, &(alt.sample_handler_task));

        vTaskCoreAffinitySet( ms5607_handle, 0x01 );
        vTaskCoreAffinitySet( adxl375_handle, 0x01 );
        vTaskCoreAffinitySet( iim42653_handle, 0x01 );

        vTaskCoreAffinitySet( alt.sample_handler_task, 0x01 );
        sampling = true;
        xTaskResumeAll();
    } else {
        printf("Stopping sample!\n");
        vTaskSuspendAll();

        vTaskDelete(ms5607_handle);
        vTaskDelete(adxl375_handle);
        vTaskDelete(iim42653_handle);

        vTaskDelete(alt.sample_handler_task);

        ms5607_handle = NULL;
        adxl375_handle = NULL;
        iim42653_handle = NULL;
        mmc5883_handle = NULL;

        alt.sample_handler_task = NULL;

        sampling = false;
        xTaskResumeAll();
    }
}


static void debug_cmd_func() {
    static TaskHandle_t logging_handle = NULL;

    if (logging_handle == NULL) {
        vTaskSuspendAll();
        xTaskCreate(logging_task, "logging", 256, NULL, LOGGING_PRIORITY, &logging_handle);
        vTaskCoreAffinitySet(logging_handle, 0x02);
        xTaskResumeAll();
    } else {
        vTaskSuspendAll();
        vTaskDelete(logging_handle);
        logging_handle = NULL;
        xTaskResumeAll();
    }
}
