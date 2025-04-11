#include <hardware/timer.h>
#include <pico/error.h>
#include <pico/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "serial.hpp"
#include "task.h"
#include "semphr.h"

#include "high_accel.hpp"
#include "altimeter.hpp"
#include "magnetometer.hpp"
#include "mid_imu.hpp"
#include "log_format.hpp"
#include "serial.hpp"

/* Priorities at which the tasks are created. */
#define EVENT_HANDLER_PRIORITY      ( tskIDLE_PRIORITY + 4 )
#define SENSOR_SAMPLE_PRIORITY      ( tskIDLE_PRIORITY + 3 )
#define	HEARTBEAT_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define LOGGING_PRIORITY            ( tskIDLE_PRIORITY + 2 )
#define	SERIAL_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

#define HEARTBEAT_RATE_HZ 5
#define DATA_UPDATE_RATE_HZ 100

#define MOVING_AVG_MAX_SIZE 20
#define MAX_SCL 400000

int input_line(char* buffer, size_t len);
void populate_log_entry();

static void sample_cmd_func();
static void debug_cmd_func();
static void logging_task(void * unused_arg);
static void update_data_task( void *pvParameters );
static void heartbeat_task( void *pvParameters );

static void update_ms5607_task( void *pvParameters );

static void update_mmc5338_task( void *pvParameters );
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

volatile int32_t altitude = 0;
volatile int32_t previous_altitude = 0;
volatile int32_t velocity = 0;
volatile state_t state = PAD;
volatile uint8_t deployment_percent = 0;

volatile bool serial_data_output = false;

volatile int32_t moving_average[MOVING_AVG_MAX_SIZE];
volatile size_t moving_average_offset = 0;
volatile size_t moving_average_size = 0;
volatile int32_t moving_average_sum = 0;
volatile SemaphoreHandle_t sensor_semaphore = NULL;

volatile int16_t ax = 0, ay = 0, az = 0;

volatile log_entry_t log_entry;

altimeter alt(i2c_default);
MidIMU mid(i2c_default);
HighAccel high(i2c_default);
Magnetometer mag(i2c_default);

extern MidIMU* log_mid;
extern HighAccel* log_high;
extern Magnetometer* log_mag;

int main() {
    log_mid = &mid;
    log_high = &high;
    log_mag = &mag;

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
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 100);

    const uint8_t adxl375_addr =  0x1D;

    uint8_t tx_buffer[2] = {0x0, 0x0};
    uint8_t rx_buffer[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    tx_buffer[0] = 0x2C; // BW_RATE REG
	tx_buffer[1] = 0x0D; // 800 Hz ODR
    i2c_write_blocking(i2c_default, adxl375_addr, tx_buffer, 2, false);

    tx_buffer[0] = 0x31; // Data Format Reg
	tx_buffer[1] = 0x0B; // Default Data Format
    i2c_write_blocking(i2c_default, adxl375_addr, tx_buffer, 2, false);

    tx_buffer[0] = 0x2D; // Power Control Reg
	tx_buffer[1] = 0x08; // Mesaurement Mode
    i2c_write_blocking(i2c_default, adxl375_addr, tx_buffer, 2, false);

    // high.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        tx_buffer[0] = 0x32;
        i2c_write_blocking(i2c_default, adxl375_addr, tx_buffer, 1, false);
        i2c_read_blocking(i2c_default, adxl375_addr, rx_buffer, 6, false);
        ax = ((int16_t) rx_buffer[0]) | ((int16_t) rx_buffer[1] << 8);
        ay = ((int16_t) rx_buffer[2]) | ((int16_t) rx_buffer[3] << 8);
        az = ((int16_t) rx_buffer[4]) | ((int16_t) rx_buffer[5] << 8);
        // high.getData();
    }
}

static void update_mmc5883_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 100);

    mag.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        mag.getData();
    }
}

static void update_iim42653_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 500);

    mid.initialize();

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        mid.getData();
    }
}

static void logging_task(void * unused_arg) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / 10);

    xLastWakeTime = xTaskGetTickCount();
    printf("Time,Pressure,Altitude,Temperature,ax,ay,az\n");
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        log_entry.time_us = time_us_64();
        log_entry.pressure = alt.get_pressure();
        log_entry.altitude = alt.get_altitude();
        log_entry.temperature_alt = alt.get_temperature();
//        log_entry.ax = mid.get_ax();
//        log_entry.ay = mid.get_ay();
//        log_entry.az = mid.get_az();
//        log_entry.gx = mid.get_gx();
//        log_entry.gy = mid.get_gy();
//        log_entry.gz = mid.get_gz();

//        log_entry.mag_x = mag.get_ax();
//        log_entry.mag_y = mag.get_ay();
//        log_entry.mag_z = mag.get_az();

//         log_entry.high_g_x = high.get_ax();
//         log_entry.high_g_y = high.get_ay();
//         log_entry.high_g_z = high.get_az();
        printf("%" PRIu64 ",", log_entry.time_us);
        printf("%4.2f,", ((float) log_entry.pressure) / PRESSURE_SCALE_F);
        printf("%4.2f,", ((float) log_entry.altitude) / ALTITUDE_SCALE_F);
        printf("%4.2f,", ((float) log_entry.temperature_alt) / TEMPERATURE_SCALE_F);
//        printf("%4.2f,", mid.scale_accel(log_entry.ax));
//        printf("%4.2f,", mid.scale_accel(log_entry.ay));
//        printf("%4.2f,", mid.scale_accel(log_entry.az));
//        
//        printf("%4.2f,", mid.scale_gyro(log_entry.gx));
//        printf("%4.2f,", mid.scale_gyro(log_entry.gy));
//        printf("%4.2f,", mid.scale_gyro(log_entry.gz));
//        
//        printf("%4.2f,", mag.scale(log_entry.mag_x));
//        printf("%4.2f,", mag.scale(log_entry.mag_y));
//        printf("%4.2f,", mag.scale(log_entry.mag_z));
        
        printf("%4.2f,", high.scale(ax));
        printf("%4.2f,", high.scale(ay));
        printf("%4.2f", high.scale(az));
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
        // xTaskCreate(update_iim42653_task, "update_iim42653", 256, NULL, SENSOR_SAMPLE_PRIORITY, &iim42653_handle);
        // xTaskCreate(update_mmc5883_task, "update_mmc5883", 256, NULL, SENSOR_SAMPLE_PRIORITY, &mmc5883_handle);

        xTaskCreate(altimeter::ms5607_sample_handler, "ms5607_sample_handler", 256, &alt, EVENT_HANDLER_PRIORITY, &(alt.sample_handler_task));

        vTaskCoreAffinitySet( ms5607_handle, 0x01 );
        vTaskCoreAffinitySet( adxl375_handle, 0x01 );
        // vTaskCoreAffinitySet( iim42653_handle, 0x01 );
        // vTaskCoreAffinitySet( mmc5883_handle, 0x01 );

        vTaskCoreAffinitySet( alt.sample_handler_task, 0x01 );
        sampling = true;
        xTaskResumeAll();
    } else {
        printf("Stopping sample!\n");
        vTaskSuspendAll();

        vTaskDelete(ms5607_handle);
        vTaskDelete(adxl375_handle);
        // vTaskDelete(iim42653_handle);
        // vTaskDelete(mmc5883_handle);

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
