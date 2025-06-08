#include <pico.h>
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
#include "math.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "serial.hpp"
#include "task.h"
#include "semphr.h"

extern "C" {
#include <fix16.h>
#include <fixmatrix.h>
}

#include "adxl375.hpp"
#include "ms5607.hpp"
#include "iim42653.hpp"
#include "mmc5983ma.hpp"
#include "pwm.hpp"
#include "pico_logger.h"
#include "log_format.hpp"
#include "serial.hpp"
#include "heartbeat.hpp"

/****************************** FREERTOS **************************************/
#define SENSOR_EVENT_HANDLER_PRIORITY   ( tskIDLE_PRIORITY + 6 )
#define SENSOR_SAMPLE_PRIORITY          ( tskIDLE_PRIORITY + 5 )
#define ROCKET_TASK_PRIORITY            ( tskIDLE_PRIORITY + 4 )
#define ROCKET_EVENT_HANDLER_PRIORITY   ( tskIDLE_PRIORITY + 3 )
#define	HEARTBEAT_TASK_PRIORITY		    ( tskIDLE_PRIORITY + 2 )
#define LOGGING_PRIORITY                ( tskIDLE_PRIORITY + 2 )
#define	SERIAL_TASK_PRIORITY		    ( tskIDLE_PRIORITY + 1 )

void vApplicationTickHook(void) { /* optional */ }
void vApplicationMallocFailedHook(void) { /* optional */ }
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) { for( ;; ); }

static void logging_task(void* pvParameters);

#define ROCKET_TASK_RATE_HZ 100
volatile TaskHandle_t rocket_task_handle = NULL;
static void rocket_task(void* pvParameters);

int64_t launch_event_callback(alarm_id_t id, void* user_data);
int64_t coast_event_callback(alarm_id_t id, void* user_data);
int64_t end_event_callback(alarm_id_t id, void* user_data);

volatile TaskHandle_t launch_event_handler_task_handle = NULL;
volatile TaskHandle_t coast_event_handler_task_handle = NULL;
volatile TaskHandle_t end_event_handler_task_handle = NULL;
static void launch_event_handler_task(void* pvParameters);
static void coast_event_handler_task(void* pvParameters);
static void end_event_handler_task(void* pvParameters);

/****************************** FREERTOS **************************************/

/****************************** SERIAL CONSOLE ********************************/
static void read_cmd_func();
static void write_cmd_func();
static void erase_cmd_func();
static void show_cmd_func();
static void deploy_cmd_func();

const char* executeable_name = "active-drag-system.uf2";
const size_t num_user_cmds = 5;
const command_t user_commands[] = { {.name = "read",
                                     .len = 4,
                                     .function = &read_cmd_func},
                                    {.name = "write",
                                     .len = 5,
                                     .function = &write_cmd_func},
                                    {.name = "erase",
                                     .len = 5,
                                     .function = &erase_cmd_func},
                                    {.name = "show",
                                     .len = 4,
                                     .function = &show_cmd_func},
                                    {.name = "deploy",
                                     .len = 6,
                                     .function = &deploy_cmd_func} };
/****************************** SERIAL CONSOLE ********************************/

/****************************** LOGGING ***************************************/
volatile bool use_circular_buffer = true;
volatile TaskHandle_t logging_handle = NULL;
volatile log_entry_t log_entry;

Logger logger(PACKET_SIZE, LOG_BASE_ADDR, print_log_entry);

static void populate_log_entry(log_entry_t* log_entry);
/****************************** LOGGING ***************************************/

volatile bool serial_data_output = false;

MS5607 alt(i2c_default);
ADXL375 adxl375(i2c_default);
IIM42653 iim42653(i2c_default);
MMC5983MA mmc5983ma(i2c_default);

PWM pwm;

#define MOTOR_BURN_TIME 6200
volatile state_t rocket_state = PAD;
volatile uint8_t deployment_percent = 0;

volatile int32_t ground_altitude = 0;
volatile int32_t threshold_altitude = 30;

volatile int32_t altitude = 0;
volatile int32_t previous_altitude = 0;

volatile int32_t velocity = 0;

#define MOVING_AVG_MAX_SIZE 100
volatile int32_t moving_average[MOVING_AVG_MAX_SIZE];
volatile size_t moving_average_offset = 0;
volatile size_t moving_average_size = 0;
volatile int32_t moving_average_sum = 0;


int main() {
    stdio_init_all();

    adc_init();
    adc_set_temp_sensor_enabled(true);

    heartbeat_initialize(PICO_DEFAULT_LED_PIN);

    i2c_init(i2c_default, 400000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    gpio_init(MICRO_DEFAULT_SERVO_ENABLE);
    gpio_set_dir(MICRO_DEFAULT_SERVO_ENABLE, GPIO_OUT);
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);

    sleep_ms(2500);

    info_cmd_func();
    stdio_flush();

    logger.initialize(true);
    logger.initialize_circular_buffer(PAD_BUFFER_SIZE);

    alt.initialize();
    sleep_ms(500);
    adxl375.initialize();
    sleep_ms(500);
    iim42653.initialize();
    sleep_ms(500);
    mmc5983ma.initialize();
    sleep_ms(500);
    pwm.init();

    fix16_t test = fix16_add(fix16_one, fix16_from_float(1.2f));
    printf("Performed 1 + 1.2 = %4.2f\n", fix16_to_float(test));

    mf16 test_mat1 = {2, 2, 0,
            {{fix16_one, fix16_from_int(12)},
             {fix16_from_float(12.54), fix16_one}}
    };

    mf16 test_mat2 = {2, 2, 0,
            {{fix16_from_float(11.11), fix16_one},
             {fix16_from_int(13), 0}}
    };

    mf16 result;

    mf16_mul(&result, &test_mat1, &test_mat2);

    printf("Resultant matrix = [%4.2f, %4.2f; %4.2f, %4.2f]\n", fix16_to_float(result.data[0][0]), fix16_to_float(result.data[0][1]), fix16_to_float(result.data[1][0]), fix16_to_float(result.data[1][1]));

    xTaskCreate(heartbeat_task, "heartbeat", 256, NULL, HEARTBEAT_TASK_PRIORITY, NULL);
    xTaskCreate(serial_task, "serial", 8192, NULL, SERIAL_TASK_PRIORITY, NULL);

    xTaskCreate(MS5607::update_ms5607_task, "update_ms5607", 1024, &alt, SENSOR_SAMPLE_PRIORITY, &(alt.update_task_handle));
    xTaskCreate(ADXL375::update_adxl375_task, "update_adxl375", 1024, &adxl375, SENSOR_SAMPLE_PRIORITY, &(adxl375.update_task_handle));
    xTaskCreate(IIM42653::update_iim42653_task, "update_iim42653", 1024, &iim42653, SENSOR_SAMPLE_PRIORITY, &(iim42653.update_task_handle));
    xTaskCreate(MMC5983MA::update_mmc5983ma_task, "update_mmc5983ma", 1024, &mmc5983ma, SENSOR_SAMPLE_PRIORITY, &(mmc5983ma.update_task_handle));

    xTaskCreate(MS5607::ms5607_sample_handler, "ms5607_sample_handler", 1024, &alt, SENSOR_EVENT_HANDLER_PRIORITY, &(alt.sample_handler_task_handle));

    xTaskCreate(rocket_task, "rocket_task", 512, NULL, SENSOR_SAMPLE_PRIORITY, const_cast<TaskHandle_t *>(&rocket_task_handle));
    vTaskCoreAffinitySet( alt.update_task_handle, 0x01 );
    vTaskCoreAffinitySet( adxl375.update_task_handle, 0x01 );
    vTaskCoreAffinitySet( iim42653.update_task_handle, 0x01 );
    vTaskCoreAffinitySet( mmc5983ma.update_task_handle, 0x01 );

    vTaskCoreAffinitySet(rocket_task_handle, 0x01);

    vTaskCoreAffinitySet( alt.sample_handler_task_handle, 0x01 );

    xTaskCreate(logging_task, "logging", 256, NULL, LOGGING_PRIORITY, const_cast<TaskHandle_t *>(&logging_handle));
    vTaskCoreAffinitySet(logging_handle, 0x02);

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
}

/****************************** LOGGING ***************************************/
static void populate_log_entry(log_entry_t* log_entry) {
    log_entry->time_us = time_us_64();

    adc_select_input(4);
    log_entry->temperature_chip = adc_read();
    log_entry->state = rocket_state;
    log_entry->deploy_percent = deployment_percent;
    log_entry->pressure = alt.get_pressure();
    log_entry->altitude = alt.get_altitude();
    log_entry->altitude_avg = altitude;
    log_entry->velocity = velocity;
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
}

static void logging_task(void* pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / LOG_RATE_HZ);

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        populate_log_entry(const_cast<log_entry_t *>(&log_entry));
        if (serial_data_output) {
            print_log_entry(reinterpret_cast<const uint8_t *>(const_cast<log_entry_t *>(&log_entry)));
            stdio_flush();
        }

        if (use_circular_buffer) {
            logger.write_circular_buffer(reinterpret_cast<const uint8_t *>(const_cast<log_entry_t *>(&log_entry)));
            if (rocket_state != PAD) {
                logger.flush_circular_buffer(true);
                use_circular_buffer = false;
            }
        } else {
            logger.write_memory(reinterpret_cast<const uint8_t *>(const_cast<log_entry_t *>(&log_entry)), false);
        }
        if ((xLastWakeTime + xFrequency) < xTaskGetTickCount()) {
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}
/****************************** LOGGING ***************************************/

/****************************** SERIAL CONSOLE ********************************/
static void read_cmd_func() {
    if (logging_handle != NULL) {
        vTaskSuspend(logging_handle);
    }
    logger.read_memory();
    if (logging_handle != NULL) {
        vTaskResume(logging_handle);
    }
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

static void show_cmd_func() {
    serial_data_output = !serial_data_output;
}

static void deploy_cmd_func() {
    vTaskSuspend(rocket_task_handle);
    printf("Enabling servo!\n");
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("Setting servo to 80%\n");
    pwm.set_servo_percent(80);
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("Setting servo to 0%\n");
    pwm.set_servo_percent(0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("Setting servo to 80%\n");
    pwm.set_servo_percent(80);
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("Setting servo to 0%\n");
    pwm.set_servo_percent(0);
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("Disabling servo!\n");
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);
    vTaskResume(rocket_task_handle);
}
/****************************** SERIAL CONSOLE ********************************/


static void rocket_task(void* pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / ROCKET_TASK_RATE_HZ);

    vTaskDelay(pdMS_TO_TICKS(1000));
    ground_altitude = alt.get_altitude();

    // Sign of life
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
    pwm.set_servo_percent(0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    pwm.set_servo_percent(20);
    vTaskDelay(pdMS_TO_TICKS(3000));
    pwm.set_servo_percent(0);
    vTaskDelay(pdMS_TO_TICKS(3000));
    pwm.set_servo_percent(20);
    vTaskDelay(pdMS_TO_TICKS(3000));
    pwm.set_servo_percent(0);
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);

    xTaskCreate(launch_event_handler_task, "launch_event_handler", 512, NULL, ROCKET_EVENT_HANDLER_PRIORITY, const_cast<TaskHandle_t *>(&launch_event_handler_task_handle));
    vTaskCoreAffinitySet(launch_event_handler_task_handle, 0x01);
    alt.set_threshold_altitude(ground_altitude + (threshold_altitude * ALTITUDE_SCALE), &launch_event_callback);

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (moving_average_size == MOVING_AVG_MAX_SIZE) {
            moving_average_sum -= moving_average[moving_average_offset];
        } else {
            moving_average_size++;
        }

        moving_average[moving_average_offset] = alt.get_altitude();
        moving_average_sum += moving_average[moving_average_offset];
        moving_average_offset = (moving_average_offset + 1) % MOVING_AVG_MAX_SIZE;

        altitude = moving_average_sum / moving_average_size;
        velocity = (altitude - previous_altitude) * ROCKET_TASK_RATE_HZ;
        previous_altitude = altitude;
        deployment_percent = 80;

        switch(rocket_state) {
            case PAD:
                deployment_percent = 0;
                pwm.set_servo_percent(deployment_percent);
                gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);
                break;
            case BOOST:
                gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
                deployment_percent = 0;
                pwm.set_servo_percent(deployment_percent);
                break;
            case COAST:
                gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
                if (velocity <= 0) {
                    rocket_state = APOGEE;
                    populate_log_entry(const_cast<log_entry_t *>(&log_entry));
                    logger.write_memory(reinterpret_cast<const uint8_t *>(const_cast<log_entry_t *>(&log_entry)), false);
                    deployment_percent = 0;
                    pwm.set_servo_percent(deployment_percent);
                    rocket_state = RECOVERY;
                    xTaskCreate(end_event_handler_task, "end_event_handler", 1024, NULL, ROCKET_EVENT_HANDLER_PRIORITY, const_cast<TaskHandle_t *>(&end_event_handler_task_handle));
                    vTaskCoreAffinitySet(end_event_handler_task_handle, 0x01);
                    add_alarm_in_ms(450000, end_event_callback, NULL, false);
                }
                pwm.set_servo_percent(deployment_percent);
                break;
            case APOGEE:
                gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
                deployment_percent = 0;
                pwm.set_servo_percent(deployment_percent);
                break;
            case RECOVERY:
                gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
                deployment_percent = 0;
                pwm.set_servo_percent(deployment_percent);
                break;
            case END:
                deployment_percent = 0;
                pwm.set_servo_percent(deployment_percent);
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);
                vTaskDelete(logging_handle);
                vTaskDelete(end_event_handler_task_handle);
                vTaskDelete(rocket_task_handle);
                break;
        }

        if ((xLastWakeTime + xFrequency) < xTaskGetTickCount()) {
            xLastWakeTime = xTaskGetTickCount();
        }
    }
}

static void launch_event_handler_task(void* pvParameters) {
    const TickType_t xInterruptFrequency = pdMS_TO_TICKS( 1000 );
    const TickType_t xMaxExpectedBlockTime = xInterruptFrequency + pdMS_TO_TICKS( 500 );
    uint32_t ulEventsToProcess;
    while (1) {
        /* Wait to receive a notification sent directly to this task from the
        interrupt service routine. */
        ulEventsToProcess = ulTaskNotifyTake( pdTRUE, xMaxExpectedBlockTime );
        if( ulEventsToProcess != 0 ) {
            /* To get here at least one event must have occurred. Loop here
            until all the pending events have been processed */
            while( ulEventsToProcess > 0 ) {
                vTaskSuspendAll();
                rocket_state = BOOST;
                xTaskCreate(coast_event_handler_task, "coast_event_handler", 512, NULL, ROCKET_EVENT_HANDLER_PRIORITY, const_cast<TaskHandle_t *>(&coast_event_handler_task_handle));
                vTaskCoreAffinitySet(coast_event_handler_task_handle, 0x01);
                add_alarm_in_ms(MOTOR_BURN_TIME, coast_event_callback, NULL, false);
                vTaskDelete(launch_event_handler_task_handle);
                launch_event_handler_task_handle = NULL;
                xTaskResumeAll();
            }
        }
    }
}

static void coast_event_handler_task(void* pvParameters) {
    const TickType_t xInterruptFrequency = pdMS_TO_TICKS( MOTOR_BURN_TIME );
    const TickType_t xMaxExpectedBlockTime = xInterruptFrequency + pdMS_TO_TICKS( 500 );
    uint32_t ulEventsToProcess;
    while (1) {
        /* Wait to receive a notification sent directly to this task from the
        interrupt service routine. */
        ulEventsToProcess = ulTaskNotifyTake( pdTRUE, xMaxExpectedBlockTime );
        if( ulEventsToProcess != 0 ) {
            /* To get here at least one event must have occurred. Loop here
            until all the pending events have been processed */
            while( ulEventsToProcess > 0 ) {
                vTaskSuspendAll();
                rocket_state = COAST;
                vTaskDelete(coast_event_handler_task_handle);
                coast_event_handler_task_handle = NULL;
                xTaskResumeAll();
            }
        }
    }
}

static void end_event_handler_task(void* pvParameters) {
    const TickType_t xInterruptFrequency = pdMS_TO_TICKS( 30000 );
    const TickType_t xMaxExpectedBlockTime = xInterruptFrequency + pdMS_TO_TICKS( 500 );

    uint32_t ulEventsToProcess;
    while (1) {
        /* Wait to receive a notification sent directly to this task from the
        interrupt service routine. */
        ulEventsToProcess = ulTaskNotifyTake( pdTRUE, xMaxExpectedBlockTime );
        if( ulEventsToProcess != 0 ) {
            /* To get here at least one event must have occurred. Loop here
            until all the pending events have been processed */
            while( ulEventsToProcess > 0 ) {
                rocket_state = END;
                vTaskDelete(end_event_handler_task_handle);
                end_event_handler_task_handle = NULL;
            }
        }
    }
}

int64_t launch_event_callback(alarm_id_t id, void* user_data) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    // Defer ISR handling to separate handler within FreeRTOS context
    vTaskNotifyGiveFromISR(launch_event_handler_task_handle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    return 0;
}

int64_t coast_event_callback(alarm_id_t id, void* user_data) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    // Defer ISR handling to separate handler within FreeRTOS context
    vTaskNotifyGiveFromISR(coast_event_handler_task_handle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    return 0;
}

int64_t end_event_callback(alarm_id_t id, void* user_data) {
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    // Defer ISR handling to separate handler within FreeRTOS context
    vTaskNotifyGiveFromISR(end_event_handler_task_handle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    return 0;
}
