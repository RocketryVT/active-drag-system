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

#include "heartbeat.hpp"
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

#define SENSOR_SAMPLE_RATE_HZ 500
#define ORIENTATION_ESTIMATION_RATE_HZ 5

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

MS5607 alt(i2c_default);
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

//TODO: This task is getting increasingly complex and involved, it may be worth migrating to a class of its own
static void pose_estimation_task(void * unused_arg) {
    printf("--POSE: INITIALIZING DATA MEMBERS\n");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / ORIENTATION_ESTIMATION_RATE_HZ);
    xLastWakeTime = xTaskGetTickCount();
    int orient_count = 0;

    Quaternionf q_k(1, 0, 0, 0);     //Initialize to straight upright
    Quaternionf q_k_est = q_k;
    Matrix4f P_k = Matrix4f::Identity()*0.01f;  //TODO: Tune this initialization value
    Matrix4f P_k_est = P_k;
    
    //TODO: Store covariance values somewhere reasonable instead of hardcoding them
    Matrix3f gyro_covariance = Matrix3f::Identity()*0.05f*M_PI/180.0f;   //0.05deg/s RMSE @ 100HZ Bandwidth
    Matrix<float, 6, 6> accel_mag_covariance;
    accel_mag_covariance << 0.00065f, 0, 0, 0, 0, 0,
                            0, 0.00065f, 0, 0, 0, 0,
                            0, 0, 0.00070f, 0, 0, 0,
                            0, 0, 0, 0.00120f, 0, 0,
                            0, 0, 0, 0, 0.00120f, 0,
                            0, 0, 0, 0, 0, 0.00120f;
    //accel_mag_covariance *= 3;

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
        
        //printf("-------- BEGINNING UPDATE COMPUTATION --------\n");
        //Get all required sensor values and compute intermediate values
        float imu_ax = -IIM42653::scale_accel(iim42653.get_ax());
        float imu_ay = -IIM42653::scale_accel(iim42653.get_ay());
        float imu_az = IIM42653::scale_accel(iim42653.get_az());
        float imu_gx = -IIM42653::scale_gyro(iim42653.get_gx())/180.0f*M_PI;
        float imu_gy = -IIM42653::scale_gyro(iim42653.get_gy())/180.0f*M_PI;
        float imu_gz = IIM42653::scale_gyro(iim42653.get_gz())/180.0f*M_PI;
        float mag_x = -MMC5983MA::scale_mag(mmc5983ma.get_ay());   //TODO: Not convinced mag axes are correct
        float mag_y = MMC5983MA::scale_mag(mmc5983ma.get_ax());
        float mag_z = -MMC5983MA::scale_mag(mmc5983ma.get_az());

        //Normalize accelerometer and magnetometer measurements
        float imu_a_mag = std::sqrt(std::pow(imu_ax, 2) + std::pow(imu_ay, 2) + std::pow(imu_az, 2));
        float mag_mag = std::sqrt(std::pow(mag_x, 2) + std::pow(mag_y, 2) + std::pow(mag_z, 2));
        imu_ax /= imu_a_mag;
        imu_ay /= imu_a_mag;
        imu_az /= imu_a_mag;
        mag_x /= mag_mag;
        mag_y /= mag_mag;
        mag_z /= mag_mag;

        printf("--- [I] INITIALIZATION | IMU Accel direct outputs (x, y, z): [%4.3f, %4.3f, %4.3f]\n", imu_ax, imu_ay, imu_az);
        printf("--- [I] INITIALIZATION | IMU Gyro direct outputs (x, y, z): [%4.3f, %4.3f, %4.3f]\n", imu_gx, imu_gy, imu_gz);
        printf("--- [I] INITIALIZATION | Magnetometer direct outputs (x, y, z): [%4.3f, %4.3f, %4.3f]\n", mag_x, mag_y, mag_z);

        float mag_D = imu_ax*mag_x + imu_ay*mag_y + imu_az*mag_z;
        float mag_N = std::sqrt(1.0f - std::pow(mag_D, 2));
        printf("--- [I] INITIALIZATION | Magnetometer values (mag_mag, mag_D, mag_N): [%2.4f, %2.4f, %2.4f]\n", mag_mag, mag_D, mag_N);
        
        //UPDATE | Calculate Kalman gain
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
        //printf("- [U] CALC KALMAN GAIN | [J_o_4_1 - J_o_4_6] after calculation, from J_observation_k: [%4.2f | %4.2f | %4.2f | %4.2f | %4.2f | %4.2f]\n", J_observation_k(3, 0), J_observation_k(3, 1), J_observation_k(3, 2), J_observation_k(3, 3), J_observation_k(3, 4), J_observation_k(3, 5));
        
        observation_noise_transformed = J_observation_k*accel_mag_covariance*J_observation_k.transpose();
        //printf("- [U] CALC KALMAN GAIN | Diagonal of observation_noise_transformed after transformation: [%2.4f, %2.4f, %2.4f, %2.4f]\n", observation_noise_transformed(0, 0), observation_noise_transformed(1, 1), observation_noise_transformed(2, 2), observation_noise_transformed(3, 3));
        
        H_k << -q_k.y(), q_k.z(), -q_k.w(), q_k.x(),
               q_k.x(), q_k.w(), q_k.z(), q_k.y(),
               q_k.w(), -q_k.x(), -q_k.y(), q_k.z(),
               q_k.z(), q_k.y(), q_k.x(), q_k.w();
        H_k *= 2;
        //printf("- [U] CALC KALMAN GAIN | First column of H_k after computation: [%2.4f, %2.4f, %2.4f, %2.4f]\n", H_k(0, 0), H_k(1, 0), H_k(2, 0), H_k(3, 0));
        
        K_k = P_k*H_k.transpose()*(H_k*P_k*H_k.transpose() + observation_noise_transformed).inverse();
        //printf("- [U] CALC KALMAN GAIN | First column of K_k after computation: [%2.4f, %2.4f, %2.4f, %2.4f]\n", K_k(0, 0), K_k(1, 0), K_k(2, 0), K_k(3, 0));

        //UPDATE | Calculate current state estimate based on accel/mag measurements
        z_k << imu_ax, imu_ay, imu_az, ((imu_ay*mag_z - imu_az*mag_y)/mag_N);
        z_k_est = H_k*q_k_est.coeffs();                 //TODO: Confirm quaternion multiplication, output and check
        q_k = q_k_est.coeffs() + K_k*(z_k - z_k_est);   //TODO: Confirm quaternion arithmetic, output and check
        q_k.normalize();    //Ensure state output is a proper rotation quaternion
        //printf("- [U] CALC STATE ESTIMATE | See below block for direct output\n");

        //UPDATE | Calculate state covariance based on accel/mag measurements
        P_k = (I_4 - K_k*H_k)*P_k_est;
        //printf("- [U] CALC STATE COVAR | First column of P_k after computation: [%2.4f, %2.4f, %2.4f, %2.4f]\n", P_k(0, 0), P_k(1, 1), P_k(2, 2), P_k(3, 3));

        //printf("-------- COMPLETED UPDATE COMPUTATION! --------\n\n");
        //if (orient_count == ORIENTATION_ESTIMATION_RATE_HZ) {
            printf("~~> Estimate (q_k) Coefficients [w, x, y, z]: [%1.4f, %1.4f, %1.4f, %1.4f]\n", q_k.w(), q_k.x(), q_k.y(), q_k.z());
            Vector3f testVect = q_k.toRotationMatrix().eulerAngles(2, 1, 0)*180.0f/M_PI;
            printf("~~> Estimate (q_k) in Euler [Yaw, Pitch, Roll]: [%3.3f, %3.3f, %3.3f]\n", testVect[0], testVect[1], testVect[2]);
            Vector3f testAccelVect(imu_ax, imu_ay, imu_az);
            Vector3f testAccelVectRotated = q_k._transformVector(testAccelVect);
            printf("~~> Estimate (q_k) rotated accel magnitude vector: [%2.4f, %2.4f, %2.4f]\n\n", testAccelVectRotated[0], testAccelVectRotated[1], testAccelVectRotated[2]);

        //    orient_count = 0;
        //}
        //orient_count++;

        //printf("-------- BEGINNING PREDICT COMPUTATION --------\n");

        //Predict next state based on gyroscope measurements
        gyro_skew << 0, -imu_gx, -imu_gy, -imu_gz,
                     imu_gx, 0, imu_gz, -imu_gy,
                     imu_gy, -imu_gz, 0, imu_gx,
                     imu_gz, imu_gy, -imu_gx, 0;
        //printf("- [P] PREDICT STATE | Top second element of gyro_skew after set: [%4.2f]\n", gyro_skew(0, 1));
        
        F_k = I_4 + (0.5f/ORIENTATION_ESTIMATION_RATE_HZ)*gyro_skew;
        //printf("- [P] PREDICT STATE | First column of F_k after set: [%2.4f, %2.4f, %2.4f, %2.4f]\n", F_k(0, 0), F_k(1, 0), F_k(2, 0), F_k(3, 0));
        
        q_k_est = F_k*q_k.coeffs();
        q_k_est.normalize();    //TODO: CONFIRM normalization necessary at intermediary steps
        //printf("- [P] PREDICT STATE | State estimate (q_k_est) coefficients after predict step: [%1.4f, %1.4f, %1.4f, %1.4f]\n", q_k_est.w(), q_k_est.x(), q_k_est.y(), q_k_est.z());

        //Predict next covariance based on gyroscope measurements + Jacobian-transformed measurement variance
        J_process_k << q_k.x(), q_k.y(), q_k.z(),
                       -1.0f*q_k.w(), q_k.z(), -1.0f*q_k.y(),
                       -1.0f*q_k.z(), -1.0f*q_k.w(), q_k.x(),
                       q_k.y(), -1.0f*q_k.x(), -1.0f*q_k.w();
        //printf("- [P] PREDICT COVAR | Top left element of J_process_k after computation: [%4.2f]\n", J_process_k(0, 0));
        
        process_noise_transformed = std::pow(0.5f/ORIENTATION_ESTIMATION_RATE_HZ, 2) *
                                    J_process_k*gyro_covariance*J_process_k.transpose();
        //printf("- [P] PREDICT COVAR | Diagonal of process_noise_transformed after transformation: [%2.4f, %2.4f, %2.4f, %2.4f]\n", process_noise_transformed(0, 0), process_noise_transformed(1, 1), process_noise_transformed(2, 2), process_noise_transformed(3, 3));
        
        P_k_est = F_k*P_k*F_k.transpose() + process_noise_transformed;
        //printf("- [P] PREDICT COVAR | Diagonal of P_k_est after prediction computation: [%2.4f, %2.4f, %2.4f, %2.4f]\n", P_k_est(0, 0), P_k_est(1, 1), P_k_est(2, 2), P_k_est(3, 3));
        //printf("-------- PREDICT COMPUTATION COMPLETED! --------\n\n\n");
    }
}

static void sample_cmd_func() {
    static bool sampling = false;

    if (sampling == false) {
        vTaskSuspendAll();

        xTaskCreate(MS5607::update_ms5607_task, "update_ms5607", 256, &alt, SENSOR_SAMPLE_PRIORITY, &(alt.update_task_handle));
        xTaskCreate(ADXL375::update_adxl375_task, "update_adxl375", 256, &adxl375, SENSOR_SAMPLE_PRIORITY, &(adxl375.update_task_handle));
        xTaskCreate(IIM42653::update_iim42653_task, "update_iim42653", 256, &iim42653, SENSOR_SAMPLE_PRIORITY, &(iim42653.update_task_handle));
        xTaskCreate(MMC5983MA::update_mmc5983ma_task, "update_mmc5983ma", 256, &mmc5983ma, SENSOR_SAMPLE_PRIORITY, &(mmc5983ma.update_task_handle));

        xTaskCreate(MS5607::ms5607_sample_handler, "ms5607_sample_handler", 256, &alt, EVENT_HANDLER_PRIORITY, &(alt.sample_handler_task_handle));

        vTaskCoreAffinitySet( alt.update_task_handle, 0x01 );
        vTaskCoreAffinitySet( adxl375.update_task_handle, 0x01 );
        vTaskCoreAffinitySet( iim42653.update_task_handle, 0x01 );
        vTaskCoreAffinitySet( mmc5983ma.update_task_handle, 0x01 );

        vTaskCoreAffinitySet( alt.sample_handler_task_handle, 0x01 );
        sampling = true;
        xTaskResumeAll();
    } else {
        printf("Stopping sample!\n");
        vTaskSuspendAll();

        vTaskDelete(alt.update_task_handle);
        vTaskDelete(adxl375.update_task_handle);
        vTaskDelete(iim42653.update_task_handle);
        vTaskDelete(mmc5983ma.update_task_handle);

        vTaskDelete(alt.sample_handler_task_handle);

        alt.update_task_handle = NULL;
        adxl375.update_task_handle = NULL;
        iim42653.update_task_handle = NULL;
        mmc5983ma.update_task_handle = NULL;

        alt.sample_handler_task_handle = NULL;

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
        printf("================ BEGINNING STATE ESTIMATION ================\n");
        vTaskSuspendAll();
        
        //TODO: Assign pose estimation unique priority?
        xTaskCreate(pose_estimation_task, "pose_estimation", 1024, NULL, 
                    SENSOR_SAMPLE_PRIORITY, &pose_estimation_handle);
        vTaskCoreAffinitySet( pose_estimation_handle, 0x01 );

        estimating = true;
        xTaskResumeAll();
    } else {
        printf("================= ENDING STATE ESTIMATION ================\n");
        vTaskSuspendAll();

        vTaskDelete(pose_estimation_handle);
        pose_estimation_handle = NULL;

        estimating = false;
        xTaskResumeAll();
    }   
}

