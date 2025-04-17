#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <cmath>    //TODO: Reimplement math properly using Eigen functions
#include <sstream>  //TODO: This is for debug purposes, remove when implemented
#include <Eigen/Dense>
using namespace Eigen;  //TODO: Limit scope to necessary components once implemented

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
#define ORIENTATION_ESTIMATION_RATE_HZ 1

#define MAX_SCL 400000

static void populate_log_entry(log_entry_t * log_entry);
static void sample_cmd_func();
static void debug_cmd_func();
static void circular_cmd_func();
static void read_cmd_func();
static void write_cmd_func();
static void erase_cmd_func();
static void orient_cmd_func();
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
const size_t num_user_cmds = 7;
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
                                     .function = &erase_cmd_func},
                                    {.name = "orient",
                                     .len = 6,
                                     .function = &orient_cmd_func} };

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

//HELPER FUNCTION FOR DEBUG, REMOVE WHEN IMPLEMENTED
/*std::string matrix2string(const Eigen::MatrixXf &mat) {
    std::stringstream ss;
    ss << mat;
    return ss.str();
}*/

//TODO: This task is getting increasingly complex and involved, it may be worth migrating to a class of its own
static void pose_estimation_task(void * unused_arg) {
    printf("--POSE: INITIALIZING DATA MEMBERS\n");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / ORIENTATION_ESTIMATION_RATE_HZ);
    xLastWakeTime = xTaskGetTickCount();
    
    Quaternionf q_k(1, 0, 0, 0);     //Initialize to straight upright
    Quaternionf q_k_prev(1, 0, 0, 0);
    Matrix4f P_k = Matrix4f::Identity()*0.01f;  //TODO: Tune this initialization value

    //TODO: Store covariance values somewhere reasonable instead of hardcoding them
    Matrix3f gyro_covariance = Matrix3f::Identity()*0.05f;   //0.05deg/s RMSE @ 100HZ Bandwidth
    Matrix<float, 6, 6> accel_mag_covariance;
    accel_mag_covariance << 0.00065f, 0, 0, 0, 0, 0,
                            0, 0.00065f, 0, 0, 0, 0,
                            0, 0, 0.00070f, 0, 0, 0,
                            0, 0, 0, 0.00120f, 0, 0,
                            0, 0, 0, 0, 0.00120f, 0,
                            0, 0, 0, 0, 0, 0.00120f;

    //Stored intermediate equation matrices
    Matrix4f gyro_skew;                     //Skew-symmetric matrix equivalent of gyroscope output
    Matrix4f I_4 = Matrix4f::Identity();    //4x4 Identity matrix, used for F_k calculation
    
    Matrix4f F_k;                           //State Transition Matrix
    Matrix4f H_k;                           //Observation Matrix
    Matrix4f K_k;                           //Kalman Gain Matrix
    Matrix<float, 4, 3> J_process_k;        //Jacobian for process (gyro) covariance transformation
    Matrix<float, 4, 6> J_observation_k;    //Jacobian for observation (accel/mag) covariance transformation
    Matrix4f process_noise_transformed;     //Matrix of transformed process noise
    Matrix4f observation_noise_transformed; //Matrix of transformed observation noise
    Matrix<float, 4, 1> z_k;                //Measurement vector
    Matrix<float, 4, 1> z_k_est;            //Estimated measurement vector, used for Kalman filter calculation
    
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        printf("-------- BEGINNING POSE COMPUTATION --------\n");
        //Get all required sensor values and compute intermediate values
        float imu_ax = -IIM42653::scale_accel(iim42653.get_ay());
        float imu_ay = -IIM42653::scale_accel(iim42653.get_ax());
        float imu_az = IIM42653::scale_accel(iim42653.get_az());
        float imu_gx = -IIM42653::scale_gyro(iim42653.get_gy());
        float imu_gy = -IIM42653::scale_gyro(iim42653.get_gx());
        float imu_gz = IIM42653::scale_gyro(iim42653.get_gz());
        float mag_x = mmc5983ma.get_ax();   //TODO: Not convinced mag axes are correct, needs testing
        float mag_y = mmc5983ma.get_ay();
        float mag_z = mmc5983ma.get_az();
        
        float mag_D = imu_ax*mag_x + imu_ay*mag_y + imu_az*mag_z;
        float mag_N = std::sqrt(1.0f - std::pow(mag_D, 2));

        //Predict next state based on gyroscope measurements
        q_k_prev = q_k;
        printf("- PREDICT STATE | First item of q_k_prev after initial set: [%4.2f]\n", q_k_prev.y());
        gyro_skew << 0, -imu_gx, -imu_gy, -imu_gz,
                     imu_gx, 0, imu_gz, -imu_gy,
                     imu_gy, -imu_gz, 0, imu_gx,
                     imu_gz, imu_gy, -imu_gx, 0;
        printf("- PREDICT STATE | Top left element of gyro_skew after set: [%4.2f]\n", gyro_skew(0, 0));
        F_k = I_4 + 0.5f/ORIENTATION_ESTIMATION_RATE_HZ*gyro_skew;
        printf("- PREDICT STATE | Top left element of F_k after set: [%4.2f]\n", F_k(0, 0));
        q_k = F_k*q_k_prev.coeffs();   //TODO: Confirm the vector conversion of this, output and check
        printf("- PREDICT STATE | First coefficient of q_k after predict step: [%4.2f]\n", q_k.w());

        //Predict next covariance based on gyroscope measurements + Jacobian-transformed measurement variance
        J_process_k << q_k_prev.x(), q_k_prev.y(), q_k_prev.z(),
                       -1.0f*q_k_prev.w(), q_k_prev.z(), -1.0f*q_k_prev.y(),
                       -1.0f*q_k_prev.z(), -1.0f*q_k_prev.w(), q_k_prev.x(),
                       q_k_prev.y(), -1.0f*q_k_prev.x(), -1.0f*q_k_prev.w();
        printf("- PREDICT COVAR | Top left element of J_process_k after computation: [%4.2f]\n", J_process_k(0, 0));
        process_noise_transformed = std::pow(0.5f/ORIENTATION_ESTIMATION_RATE_HZ, 2) *
                                    J_process_k*gyro_covariance*J_process_k.transpose();
        printf("- PREDICT COVAR | Top left element of process_noise_transformed after transformation: [%4.2f]\n", process_noise_transformed(0, 0));
        P_k = F_k*P_k*F_k.transpose() + process_noise_transformed;
        printf("- PREDICT COVAR | Top left element of P_k after computation: [%4.2f]\n", P_k(0, 0));

        //Calculate Kalman gain
        //TODO: This Jacobian calculation is FUGLY, and very computationally intensive
        //   Separate out common math, vectorize, and put it in a function somewhere?
        float pd_mD_ax = mag_x/(2*mag_D);
        float pd_mD_ay = mag_y/(2*mag_D);
        float pd_mD_az = mag_z/(2*mag_D);
        float pd_mD_mx = imu_ax/(2*mag_D);
        float pd_mD_my = imu_ay/(2*mag_D);
        float pd_mD_mz = imu_az/(2*mag_D);
        float pd_mN_ax = -mag_D/mag_N*pd_mD_ax;
        float pd_mN_ay = -mag_D/mag_N*pd_mD_ay;
        float pd_mN_az = -mag_D/mag_N*pd_mD_az;
        float pd_mN_mx = -mag_D/mag_N*pd_mD_mx;
        float pd_mN_my = -mag_D/mag_N*pd_mD_my;
        float pd_mN_mz = -mag_D/mag_N*pd_mD_mz;
        float common_num = (imu_ay*mag_z - imu_az*mag_y);
        float common_denom = std::pow(mag_N, 2);

        float J_o_4_1 = (-pd_mN_ax*common_num)/common_denom;
        float J_o_4_2 = (mag_z*mag_N - pd_mN_ay*common_num)/common_denom;
        float J_o_4_3 = (-mag_y*mag_N - pd_mN_az*common_num)/common_denom;
        float J_o_4_4 = (-pd_mN_mx*common_num)/common_denom;
        float J_o_4_5 = (-imu_az*mag_N - pd_mN_my*common_num)/common_denom;
        float J_o_4_6 = (imu_ay*mag_N - pd_mN_mz*common_num)/common_denom;
        J_observation_k << 1.0f, 0, 0, 0, 0, 0,
                           0, 1.0f, 0, 0, 0, 0,
                           0, 0, 1.0f, 0, 0, 0,
                           J_o_4_1, J_o_4_2, J_o_4_3, J_o_4_4, J_o_4_5, J_o_4_6;
        printf("- CALC KALMAN GAIN | [J_o_4_1 - J_o_4_6] after calculation, from J_observation_k: [%4.2f | %4.2f | %4.2f | %4.2f | %4.2f | %4.2f]\n", J_observation_k(3, 0), J_observation_k(3, 1), J_observation_k(3, 2), J_observation_k(3, 3), J_observation_k(3, 4), J_observation_k(3, 5));
        observation_noise_transformed = J_observation_k*accel_mag_covariance*J_observation_k.transpose();
        printf("- CALC KALMAN GAIN | Top left element of observation_noise_transformed after computation: [%4.2f]\n", observation_noise_transformed(0, 0));
        H_k << -q_k.y(), q_k.z(), -q_k.w(), q_k.x(),
               q_k.x(), q_k.w(), q_k.z(), q_k.y(),
               q_k.w(), -q_k.x(), -q_k.y(), q_k.z(),
               q_k.z(), q_k.y(), q_k.x(), q_k.w();
        H_k *= 2;
        printf("- CALC KALMAN GAIN | Top left element of H_k after computation: [%4.2f]\n", H_k(0, 0));
        K_k = P_k*H_k.transpose()*(H_k*P_k*H_k.transpose() + observation_noise_transformed).inverse();
        printf("- CALC KALMAN GAIN | Top left element of K_k after computation: [%4.2f]\n", K_k(0, 0));

        //Update state estimate based on accel/mag measurements
        z_k << imu_ax, imu_ay, imu_az, ((imu_ay*mag_z - imu_az*mag_y)/mag_N);
        z_k_est = H_k*q_k.coeffs();                 //TODO: Confirm quaternion multiplication, output and check
        q_k = q_k.coeffs() + K_k*(z_k - z_k_est);   //TODO: Confirm quaternion arithmetic, output and check
        q_k.normalize();    //Ensure state output is a proper rotation quaternion

        //Update covariance based on accel/mag measurements
        P_k = (I_4 - K_k*H_k)*P_k;
        printf("COMPLETED COMPUTATION!\n");
        
        printf("Estimate (q_k) Coefficients [w, x, y, z]: [%4.2f, %4.2f, %4.2f, %4.2f]\n", q_k.w(), q_k.x(), q_k.y(), q_k.z());
        Vector3f testVect = q_k._transformVector(Vector3f::UnitZ());
        printf("Estimate (q_k) Rotated [X, Y, Z]: [%4.2f, %4.2f, %4.2f]\n\n", testVect[0], testVect[1], testVect[2]);
    }
}

static void sample_cmd_func() {
    static TaskHandle_t ms5607_handle    = NULL;
    static TaskHandle_t adxl375_handle   = NULL;
    static TaskHandle_t iim42653_handle  = NULL;
    static TaskHandle_t mmc5983ma_handle = NULL;
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

static void orient_cmd_func() {
    /*
    float pitch_rad = std::atan(imu_ay/std::sqrt(std::pow(imu_ax, 2) + std::pow(imu_az, 2)));
    float roll_rad = std::atan(imu_ax/std::sqrt(std::pow(imu_ay, 2) + std::pow(imu_az, 2)));
    float yaw_rad = std::atan2(-mag_y*std::cos(roll_rad) + mag_z*std::sin(roll_rad), 
            mag_x*std::cos(pitch_rad) + mag_y*std::sin(pitch_rad)*std::sin(roll_rad) + mag_z*std::cos(roll_rad) * std::sin(pitch_rad));
    */

    static TaskHandle_t pose_estimation_handle = NULL;
    static bool estimating = false;

    if (!estimating) {
        printf("======== BEGINNING STATE ESTIMATION ========\n");
        vTaskSuspendAll();
        
        //TODO: Assign pose estimation unique priority?
        xTaskCreate(pose_estimation_task, "pose_estimation", 1024, NULL, 
                    SENSOR_SAMPLE_PRIORITY, &pose_estimation_handle);
        vTaskCoreAffinitySet( pose_estimation_handle, 0x01 );

        estimating = true;
        xTaskResumeAll();
    } else {
        printf("======== ENDING STATE ESTIMATION ========\n");
        vTaskSuspendAll();

        vTaskDelete(pose_estimation_handle);
        pose_estimation_handle = NULL;

        estimating = false;
        xTaskResumeAll();
    }   
}

