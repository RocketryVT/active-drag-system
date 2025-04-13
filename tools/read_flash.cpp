#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include <hardware/timer.h>
#include "pico/rand.h"
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
#include "mmc5983ma.hpp"
#include "serial.hpp"
#include "pico_logger.h"
#include "log_format.hpp"

/* Priorities at which the tasks are created. */
#define EVENT_HANDLER_PRIORITY      ( tskIDLE_PRIORITY + 4 )
#define SENSOR_SAMPLE_PRIORITY      ( tskIDLE_PRIORITY + 3 )
#define	HEARTBEAT_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define LOGGING_PRIORITY            ( tskIDLE_PRIORITY + 2 )
#define	SERIAL_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

#define HEARTBEAT_RATE_HZ 5
#define SENSOR_SAMPLE_RATE_HZ 500

#define MAX_SCL 400000

static void populate_log_entry(log_entry_t * log_entry);
static void sample_cmd_func();
static void debug_cmd_func();
static void circular_cmd_func();
static void read_cmd_func();
static void write_cmd_func();
static void erase_cmd_func();
static void logging_task(void * unused_arg);
static void update_data_task( void *pvParameters );
static void heartbeat_task( void *pvParameters );

static void update_ms5607_task( void *pvParameters );

static void update_adxl375_task( void *pvParameters );
static void update_iim42653_task( void *pvParameters );
static void update_mmc5983ma_task( void *pvParameters );

void vApplicationTickHook(void) { /* optional */ }
void vApplicationMallocFailedHook(void) { /* optional */ }
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) { for( ;; ); }

const char* executeable_name = "read_flash.uf2";
const size_t num_user_cmds = 6;
const command_t user_commands[] = { {.name = "sample",
                                     .len = 6,
                                     .function = &sample_cmd_func},
                                    {.name = "debug",
                                     .len = 5,
                                     .function = &debug_cmd_func},
                                    {.name = "circular",
                                     .len = 8,
                                     .function = &circular_cmd_func},
                                    {.name = "read",
                                     .len = 4,
                                     .function = &read_cmd_func},
                                    {.name = "write",
                                     .len = 5,
                                     .function = &write_cmd_func},
                                    {.name = "erase",
                                     .len = 5,
                                     .function = &erase_cmd_func} };

volatile bool serial_data_output = false;
volatile bool use_circular_buffer = false;

altimeter alt(i2c_default);
ADXL375 adxl375(i2c_default);
IIM42653 iim42653(i2c_default);
MMC5983MA mmc5983ma(i2c_default);

Logger logger(PACKET_SIZE, LOG_BASE_ADDR, print_log_entry);

volatile TaskHandle_t logging_handle = NULL;

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

    logger.initialize(true);

    alt.initialize();
    sleep_ms(500);
    adxl375.initialize();
    sleep_ms(500);
    iim42653.initialize();
    sleep_ms(500);
    mmc5983ma.initialize();
    sleep_ms(500);

    xTaskCreate(heartbeat_task, "heartbeat", 256, NULL, HEARTBEAT_TASK_PRIORITY, NULL);
    xTaskCreate(serial_task, "serial", 8192, NULL, SERIAL_TASK_PRIORITY, NULL);

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
}

static void update_ms5607_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_SAMPLE_RATE_HZ);

    // alt.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        alt.ms5607_start_sample();
    }
}

static void update_adxl375_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_SAMPLE_RATE_HZ);

    // adxl375.initialize();

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
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / SENSOR_SAMPLE_RATE_HZ);

    // iim42653.initialize();

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

static void update_mmc5983ma_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 200);

    // mmc5983ma.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        taskENTER_CRITICAL();
        mmc5983ma.sample();
        mmc5983ma.apply_offset();
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
    printf("Time,Pressure,Altitude,Temperature,ax,ay,az,ax,ay,az,gx,gy,gz,ax,ay,az\n");
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//        printf("%" PRIu64 ",", time_us_64());
//        printf("%4.2f,", ((float) alt.get_pressure()) / PRESSURE_SCALE_F);
//        printf("%4.2f,", ((float) alt.get_altitude()) / ALTITUDE_SCALE_F);
//        printf("%4.2f,", ((float) alt.get_temperature()) / TEMPERATURE_SCALE_F);
//        printf("%4.2f,", adxl375.scale(adxl375.get_ax()));
//        printf("%4.2f,", adxl375.scale(adxl375.get_ay()));
//        printf("%4.2f,", adxl375.scale(adxl375.get_az()));
//        printf("%4.2f,", iim42653.scale_accel(iim42653.get_ax()));
//        printf("%4.2f,", iim42653.scale_accel(iim42653.get_ay()));
//        printf("%4.2f,", iim42653.scale_accel(iim42653.get_az()));
//        printf("%4.2f,", iim42653.scale_gyro(iim42653.get_gx()));
//        printf("%4.2f,", iim42653.scale_gyro(iim42653.get_gy()));
//        printf("%4.2f,", iim42653.scale_gyro(iim42653.get_gz()));
//        printf("%" PRIi16 ",", mmc5983ma.get_ax());
//        printf("%" PRIi16 ",", mmc5983ma.get_ay());
//        printf("%" PRIi16 ",", mmc5983ma.get_az());
//        printf("\r\n");
        log_entry_t log_entry;
        populate_log_entry(&log_entry);
        printf("\nWriting the following entry!\n");
        print_log_entry(reinterpret_cast<const uint8_t *>(&log_entry));
        if (use_circular_buffer) {
            logger.write_circular_buffer(reinterpret_cast<const uint8_t *>(&log_entry));
        } else {
            logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), true);
        }
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
    static TaskHandle_t mmc5983ma_handle  = NULL;
    static bool sampling = false;

    if (sampling == false) {
        vTaskSuspendAll();

        xTaskCreate(update_ms5607_task, "update_ms5607", 256, NULL, SENSOR_SAMPLE_PRIORITY, &ms5607_handle);
        xTaskCreate(update_adxl375_task, "update_adxl375", 256, NULL, SENSOR_SAMPLE_PRIORITY, &adxl375_handle);
        xTaskCreate(update_iim42653_task, "update_iim42653", 256, NULL, SENSOR_SAMPLE_PRIORITY, &iim42653_handle);
        xTaskCreate(update_mmc5983ma_task, "update_mmc5983ma", 256, NULL, SENSOR_SAMPLE_PRIORITY, &mmc5983ma_handle);

        xTaskCreate(altimeter::ms5607_sample_handler, "ms5607_sample_handler", 256, &alt, EVENT_HANDLER_PRIORITY, &(alt.sample_handler_task));

        vTaskCoreAffinitySet( ms5607_handle, 0x01 );
        vTaskCoreAffinitySet( adxl375_handle, 0x01 );
        vTaskCoreAffinitySet( iim42653_handle, 0x01 );
        vTaskCoreAffinitySet( mmc5983ma_handle, 0x01 );

        vTaskCoreAffinitySet( alt.sample_handler_task, 0x01 );
        sampling = true;
        xTaskResumeAll();
    } else {
        printf("Stopping sample!\n");
        vTaskSuspendAll();

        vTaskDelete(ms5607_handle);
        vTaskDelete(adxl375_handle);
        vTaskDelete(iim42653_handle);
        vTaskDelete(mmc5983ma_handle);

        vTaskDelete(alt.sample_handler_task);

        ms5607_handle = NULL;
        adxl375_handle = NULL;
        iim42653_handle = NULL;
        mmc5983ma_handle = NULL;

        alt.sample_handler_task = NULL;

        sampling = false;
        xTaskResumeAll();
    }
}


static void debug_cmd_func() {
    if (logging_handle == NULL) {
        vTaskSuspendAll();
        xTaskCreate(logging_task, "logging", 256, NULL, LOGGING_PRIORITY, const_cast<TaskHandle_t *>(&logging_handle));
        vTaskCoreAffinitySet(logging_handle, 0x02);
        xTaskResumeAll();
    } else {
        vTaskSuspendAll();
        vTaskDelete(logging_handle);
        logging_handle = NULL;
        xTaskResumeAll();
    }
}

static void circular_cmd_func() {
    if (logging_handle != NULL) {
        vTaskSuspend(logging_handle);
    }
    if (!use_circular_buffer) {
        logger.initialize_circular_buffer(PAD_BUFFER_SIZE);
        use_circular_buffer = true;
    } else {
        logger.flush_circular_buffer(true);
        use_circular_buffer = false;
    }
    if (logging_handle != NULL) {
        vTaskResume(logging_handle);
    }
}

static void read_cmd_func() {
    if (logging_handle != NULL) {
        vTaskSuspend(logging_handle);
    }
    if (use_circular_buffer) {
        logger.read_circular_buffer();
    } else {
        logger.read_memory();
    }
    vTaskResume(logging_handle);
}

static void populate_log_entry(log_entry_t * log_entry) {
    log_entry->time_us = time_us_64();

    adc_select_input(4);
    log_entry->temperature_chip = adc_read();
    log_entry->state = PAD;
    log_entry->deploy_percent = 80;
    log_entry->pressure = alt.get_pressure();
    log_entry->altitude = alt.get_altitude();
    log_entry->temperature_alt = alt.get_temperature();

    log_entry->ax = iim42653.get_ax();
    log_entry->ay = iim42653.get_ay();
    log_entry->az = iim42653.get_az();
    log_entry->gx = iim42653.get_gx();
    log_entry->gy = iim42653.get_gy();
    log_entry->gz = iim42653.get_gz();

    log_entry->mag_x = mmc5983ma.get_ax();
    log_entry->mag_y = mmc5983ma.get_ay();
    log_entry->mag_z = mmc5983ma.get_az();

    log_entry->high_g_x = adxl375.get_ax();
    log_entry->high_g_y = adxl375.get_ay();
    log_entry->high_g_z = adxl375.get_az();

    log_entry->data0 = get_rand_64();
    log_entry->data1 = get_rand_64();
    log_entry->data2 = get_rand_32();
    log_entry->data3 = get_rand_32();
}

static void write_cmd_func() {
    if (logging_handle != NULL) {
        vTaskSuspend(logging_handle);
    }
    uint64_t start = time_us_64();
    log_entry_t log_entry;
    populate_log_entry(&log_entry);
    printf("\nWriting the following entry!\n");
    print_log_entry(reinterpret_cast<const uint8_t *>(&log_entry));
    if (use_circular_buffer) {
        logger.write_circular_buffer(reinterpret_cast<const uint8_t *>(&log_entry));
    } else {
        logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), true);
    }
    uint64_t end = time_us_64();
    printf("\nTook %" PRIu64 " us to write that entry!\n", (end - start));
    if (logging_handle != NULL) {
        vTaskResume(logging_handle);
    }
}

static void erase_cmd_func() {
    if (logging_handle != NULL) {
        vTaskSuspend(logging_handle);
    }
    logger.erase_memory();
    if (logging_handle != NULL) {
        vTaskResume(logging_handle);
    }
}
