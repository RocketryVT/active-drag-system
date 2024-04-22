/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <algorithm>
#include <stdio.h>
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/sem.h"
#include "spi_flash.h"
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include <math.h>

#include "bno055.hpp"
#include "AltEst/altitude.h"
#include "pwm.hpp"
#include "SimpleKalmanFilter.h"

#define ALT_ADDR 0x60
#define MAX_SCL 400000
#define DATA_RATE_HZ 100
#define LOOP_PERIOD (1.0f / DATA_RATE_HZ)
#define INT1_PIN 6 // INT1 PIN on MPL3115A2 connected to GPIO PIN 9 (GP6)
#define MOSFET_PIN 1 // MOSFET PIN connected to GPIO PIN 1 (GP1)

#define GRAVITY -9.81
#define LOG_RATE_HZ 4
#define INT1_PIN 6 // INT1 PIN on MPL3115A2 connected to GPIO PIN 9 (GP6)
#define LED_PIN 28

#define MOTOR_BURN_TIME 6200 // Burn time in milliseconds for M1939
typedef enum {
    PAD,
    BOOST,
    COAST,
    APOGEE,
    RECOVERY,
    END
} state_t;

BNO055 bno055;
PWM pwm;
static AltitudeEstimator vKF = AltitudeEstimator(0.0005, // sigma Accel
                                                      0.0005, // sigma Gyro
                                                      0.018,   // sigma Baro
                                                      0.5, // ca
                                                      0.1);// accelThreshold

void pad_callback(uint gpio, uint32_t event_mask);
int64_t boost_callback(alarm_id_t id, void* user_data);
int64_t apogee_callback(alarm_id_t id, void* user_data);
int64_t coast_callback(alarm_id_t id, void* user_data);
void recovery_callback(uint gpio, uint32_t event_mask);
void init_altimeter();
float get_deploy_percent(float velocity, float altitude);

bool timer_callback(repeating_timer_t *rt);
bool test_timer_callback(repeating_timer_t *rt);

float get_altitude();
float get_velocity();

void snapshot();
bool logging_callback(repeating_timer_t *rt);
void logging_core();

semaphore_t sem;

volatile float altitude = 0.0f;
volatile float prev_altitude = 0.0f;
volatile float velocity = 0.0f;
volatile state_t state = PAD;
volatile float threshold_altitude = 30.0f;
volatile float threshold_velocity = 30.0f;
volatile uint8_t deployment_percent = 0;

volatile vector3f linear_acceleration;
volatile vector3f acceleration;
volatile quarternion abs_quaternion;
volatile vector3f velocity_vector;

volatile vector3f euler_angles;
volatile vector3f abs_lin_accel;
volatile vector3f prev_abs_lin_accel;
volatile vector3f rot_y_vec;
volatile vector3f vel_at_angle;

volatile vector3f accel_gravity;

volatile CALIB_STATUS calib_status;
uint8_t accel[6];
uint8_t quat[8];

repeating_timer_t data_timer;
repeating_timer_t log_timer;

float ground_altitude = 0.0f;

SimpleKalmanFilter altitudeKF(2, 2, 0.01);
SimpleKalmanFilter velocityKF(1, 1, 0.01);

/**
 * @brief Main function
 * 
 * @return int erorr code
 */
int main() {
    // stdio_init_all();

    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Enable SPI 0 at 60 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 1000 * 60);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_init(INT1_PIN);
    gpio_pull_up(INT1_PIN);
    
    alarm_pool_init_default();


    // Initialize altimeter
    init_altimeter();

    // Initialize BNO055
    bno055.init();

    // Initialize PWM
    pwm.init();

    // Initialize MOSFET
    gpio_init(MOSFET_PIN);
    gpio_set_dir(MOSFET_PIN, GPIO_OUT);

    // Initialize Kalman Filter
    float measurement = get_altitude();
    float v_measurement = get_velocity();
    altitudeKF.updateEstimate(measurement);
    velocityKF.updateEstimate(measurement);

    ground_altitude = altitude;

    sem_init(&sem, 1, 1);

    if (!add_repeating_timer_us(-1000000 / DATA_RATE_HZ,  &timer_callback, NULL, &data_timer)) {
        // printf("Failed to add timer!\n");
        return -1;
    }

    multicore_launch_core1(logging_core);

    while (1) {
        tight_loop_contents();
    }
}

void logging_core() {
    add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_callback, NULL, &log_timer);

    while (1) {
        tight_loop_contents();
    }
}

/**
 * @brief Initializes the altimeter
 * 
 * @param altitude passes the altitude variable to the function
 * @param threshold_altitude passes the threshold altitude variable to the function
 */
void init_altimeter() {

    uint8_t config[2] = {0};

    // Select control register(0x26)
    // Active mode, OSR = 16, altimeter mode(0xB8)
    config[0] = 0x26;
    config[1] = 0x89;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select data configuration register(0x13)
    // Data ready event enabled for altitude, pressure, temperature(0x07)
    // config[0] = 0x13;
    // config[1] = 0x07;
    // i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Below configures the interrupt for the first transition from PAD to BOOST
    // Initial Reading

    sleep_ms(1000);

    while (altitude == 0.0f) {
        altitude = get_altitude();
    }

    threshold_altitude += altitude; // 30 meters above ground

    // printf("threshold_altitude: %4.2f", threshold_altitude);

    // Select control register 3 (0x28)
    // Set bot interrupt pins to active low and enable internal pullups
    config[0] = 0x28;
    config[1] = 0x01;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select pressure target MSB register(0x16)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x16;
    config[1] = (uint8_t) (((int16_t)(threshold_altitude)) >> 8);
    // printf("threshold_alt upper half: %X\n", config[1]);
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select pressure target LSB register(0x17)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x17;
    config[1] = (uint8_t) (((int16_t)(threshold_altitude)));
    // printf("threshold_alt lower half: %X\n", config[1]);
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    config[0] = 0x29;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    config[0] = 0x2A;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_LEVEL_LOW, true, &pad_callback);
    // End of configuration of interrupt for first transition from PAD to BOOST
}

void snapshot() {
    if (state != END) {
        uint8_t entry[PACKET_SIZE];
        absolute_time_t now = get_absolute_time();
        uint64_t now_us = to_us_since_boot(now);
        uint32_t alt_bits = *((uint32_t *)&altitude);
        uint32_t vel_bits = *((uint32_t *)&velocity);
        uint32_t acc_bits = *((uint32_t *)&abs_lin_accel.z);
        entry[0] = now_us >> 56;
        entry[1] = now_us >> 48;
        entry[2] = now_us >> 40;
        entry[3] = now_us >> 32;
        entry[4] = now_us >> 24;
        entry[5] = now_us >> 16;
        entry[6] = now_us >> 8;
        entry[7] = now_us;

        switch (state) {
            case PAD:
                entry[8] = 'P';
                break;
            case BOOST:
                entry[8] = 'B';
                break;
            case COAST:
                entry[8] = 'C';
                break;
            case APOGEE:
                entry[8] = 'A';
                break;
            case RECOVERY:
                entry[8] = 'R';
                break;
            case END:
                entry[8] = 'E';
                break;
        }

        entry[9] = deployment_percent;
        entry[10] = alt_bits >> 24;
        entry[11] = alt_bits >> 16;
        entry[12] = alt_bits >> 8;
        entry[13] = alt_bits;
        entry[14] = vel_bits >> 24;
        entry[15] = vel_bits >> 16;
        entry[16] = vel_bits >> 8;
        entry[17] = vel_bits;

        entry[18] = quat[0];
        entry[19] = quat[1];
        entry[20] = quat[2];
        entry[21] = quat[3];
        entry[22] = quat[4];
        entry[23] = quat[5];
        entry[24] = quat[6];
        entry[25] = quat[7];

        entry[26] = acc_bits >> 24;
        entry[27] = acc_bits >> 16;
        entry[28] = acc_bits >> 8;
        entry[29] = acc_bits;
        entry[30] = accel[4];
        entry[31] = accel[5];
        write_entry(entry);
    }
}

bool logging_callback(repeating_timer_t *rt) {
    sem_acquire_blocking(&sem);
    snapshot();
    sem_release(&sem);
    return true;
}

bool timer_callback(repeating_timer_t *rt) {
    absolute_time_t last = get_absolute_time();
    sem_acquire_blocking(&sem);
    float measurement = get_altitude();
    altitude = altitudeKF.updateEstimate(measurement);
    // float in_velocity = (altitude - prev_altitude) * DATA_RATE_HZ;
    // velocity = velocityKF.updateEstimate(in_velocity);
    float acceldata[3];
    float gyrodata[3];


    // printf("Velocity_Delta: %4.2f\tVelocity_Prev: %4.2f\n", velocity, ((altitude - prev_altitude) * DATA_RATE_HZ));

    bno055.read_lin_accel();
    // printf("Linear Acceleration:\n" "x: %f\n" "y: %f\n" "z: %f\n",
    //  linear_acceleration.x, linear_acceleration.y, linear_acceleration.z);

    bno055.read_abs_quaternion();
    // printf("Absolute Quaternion:\n" "w: %f\n" "x: %f\n" "y: %f\n" "z: %f\n",
    //  abs_quaternion.w, abs_quaternion.x, abs_quaternion.y, abs_quaternion.z);

    bno055.read_euler_angles();
    // printf("Euler Angles:\n" "Roll: %f\n" "Pitch: %f\n" "Yaw: %f\n",
    //  euler_angles.x, euler_angles.y, euler_angles.z);

    // Linear Acceleration and Absolute Quaternion are used to calculate Absolute Linear Acceleration
    // They must be read before calling this function
    bno055.calculate_abs_linear_acceleration();
    // printf("Absolute Linear Acceleration:\n" "x: %f\n" "y: %f\n" "z: %f\n",
    // abs_lin_accel.x, abs_lin_accel.y, abs_lin_accel.z);
    
    acceldata[0] = abs_lin_accel.x;
    acceldata[1] = abs_lin_accel.y;
    acceldata[2] = abs_lin_accel.z - 0.4f;
    gyrodata[0] = 0;
    gyrodata[1] = 0;
    gyrodata[2] = 0;

    vKF.estimate(acceldata, gyrodata, altitude, to_us_since_boot(last));
    velocity = vKF.getVerticalVelocity();
    // printf("Measurement: %4.2f, Altitude: %4.2f, Velocity: %4.2f\n", measurement, altitude, velocity);
    // This is wrong but i'm going home.
    // velocity_vector.x = (prev_abs_lin_accel.x - abs_lin_accel.x) / 0.01f);
    // velocity_vector.y = (prev_abs_lin_accel.y - abs_lin_accel.y) / 0.01f);
    // velocity_vector.z = (prev_abs_lin_accel.z - abs_lin_accel.z) / 0.01f);
    // printf("Velocity Vector:\n" "x: %f\n" "y: %f\n" "z: %f\n",
    // velocity_vector.x, velocity_vector.y, velocity_vector.z);

    prev_abs_lin_accel.x = abs_lin_accel.x;
    prev_abs_lin_accel.y = abs_lin_accel.y;
    prev_abs_lin_accel.z = abs_lin_accel.z;

    bno055.accel_to_gravity();
    float agl = altitude - ground_altitude;

    deployment_percent = (uint8_t)(std::min(std::max(30.0f, get_deploy_percent(velocity, agl)), 100.0f));

    switch(state) {
        case PAD:
            gpio_put(MOSFET_PIN, 0);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case BOOST:
            gpio_put(MOSFET_PIN, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case COAST:
            gpio_put(MOSFET_PIN, 1);
            pwm.set_servo_percent(deployment_percent);
            break;
        case APOGEE:
            gpio_put(MOSFET_PIN, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case RECOVERY:
            gpio_put(MOSFET_PIN, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case END:
            gpio_put(MOSFET_PIN, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
    }
    prev_altitude = altitude;
    sem_release(&sem);
    return true;
}

/**
 * @brief Test function for timer callback outputs data in ROS2 format
 * 
 * @param rt 
 * @return true
 * @return false 
 */
// bool test_timer_callback(repeating_timer_t *rt) {
//     static float prev_altitude = altitude;
//     absolute_time_t last = get_absolute_time();
//     altitude = get_altitude();
//     velocity = ((altitude - prev_altitude) / 0.01f);
//     prev_altitude = altitude;
// 
//     bno055.read_lin_accel();
//     bno055.read_abs_quaternion();
// 
//     absolute_time_t now = get_absolute_time();
//     int64_t time_delta = absolute_time_diff_us(last, now);
// 
//     std::cout << altitude << " " << abs_quaternion.w << " "
//           << abs_quaternion.x << " "
//           << abs_quaternion.y << " "
//           << abs_quaternion.z << " "
//           << linear_acceleration.x << " "
//           << linear_acceleration.y << " "
//           << linear_acceleration.z << std::endl;
// 
//     /* switch (state) {
//         case PAD:
//             printf("P\n");
//             break;
//         case BOOST:
//             printf("B\n");
//             break;
//         case COAST:
//             printf("C\n");
//             break;
//         case APOGEE:
//             printf("A\n");
//             break;
//         case RECOVERY:
//             printf("R\n");
//             break;
//         case END:
//             printf("E\n");
//             break;
//     }*/
// 
//     // absolute_time_t now = get_absolute_time();
//     // int64_t time_delta = absolute_time_diff_us(last, now);
//     // printf("Time Delta: %" PRIi64"\n", time_delta);
//     // std::flush(std::cout);
//     prev_altitude = altitude;
//     return true;
// }

/**
 * @brief Call back function for when rocket is on the pad
 * 
 * @param gpio pin number of interrupt
 * @param event_mask interrupt condition, value is set by PICO_SDK
 *  GPIO_IRQ_LEVEL_LOW = 0x1u,
 *  GPIO_IRQ_LEVEL_HIGH = 0x2u,
 *  GPIO_IRQ_EDGE_FALL = 0x4u,
 *  GPIO_IRQ_EDGE_RISE = 0x8u,
 * @link https://www.raspberrypi.com/documentation/pico-sdk/hardware/gpio.html#ga6347e27da3ab34f1ea65b5ae16ab724f
 */
void pad_callback(uint gpio, uint32_t event_mask) {

    /// @link https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#ga6165f07f4b619dd08ea6dc97d069e78a
    /// Each pin only supports one call back, so by calling this we overwrite the previous one
    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_LEVEL_LOW, false, &pad_callback);
    uint8_t config[2] = {0};
    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    config[0] = 0x29;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    config[0] = 0x2A;
    config[1] = 0x00;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    sem_acquire_blocking(&sem);
    state = BOOST;
    // start motor burn timer with this function as callback
    add_alarm_in_ms(MOTOR_BURN_TIME, &boost_callback, NULL, false);
    snapshot();
    sem_release(&sem);
}

int64_t boost_callback(alarm_id_t id, void* user_data) {
    // Configure accelerometer and/or altimeter to generate interrupt
    // for when velocity is negative with this function as callback to
    // transition to APOGEE
    sem_acquire_blocking(&sem);
    state = COAST;
    snapshot();
    sem_release(&sem);
    add_alarm_in_ms(1000, &coast_callback, NULL, false);
    return 0;
}

int64_t coast_callback(alarm_id_t id, void* user_data) {
    // Want to somehow immediately transition to RECOVERY from APOGEE (extremely short timer?)
    if (velocity <= 0.0f) {
        sem_acquire_blocking(&sem);
        state = APOGEE;
        snapshot();
        sem_release(&sem);
        add_alarm_in_ms(1, &apogee_callback, NULL, false);
    } else {
        add_alarm_in_ms(250, &coast_callback, NULL, false);
    }
    return 0;
}

int64_t apogee_callback(alarm_id_t id, void* user_data) {
    // Set altimeter interrupt to occur for when rocket touches back to the ground

    uint8_t config[2] = {0};
    // Select pressure target MSB register(0x16)
    // Set altitude target to 10 meters above ground altitude
    float recov_altitude = ground_altitude + 10.0f;
    // Select pressure target MSB register(0x16)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x16;
    config[1] = (uint8_t) (((int16_t)(recov_altitude)) >> 8);
    // printf("threshold_alt upper half: %X\n", config[1]);
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select pressure target LSB register(0x17)
    // Set altitude target to 30 meters above ground altitude
    config[0] = 0x17;
    config[1] = (uint8_t) (((int16_t)(recov_altitude)));
    // printf("threshold_alt lower half: %X\n", config[1]);
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    config[0] = 0x29;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    config[0] = 0x2A;
    config[1] = 0x08;
    i2c_write_blocking(i2c_default, ALT_ADDR, config, 2, true);

    sem_acquire_blocking(&sem);
    state = RECOVERY;
    snapshot();
    sem_release(&sem);
    gpio_set_irq_enabled_with_callback(INT1_PIN, GPIO_IRQ_LEVEL_LOW, true, &recovery_callback);
    return 0;
}

void recovery_callback(uint gpio, uint32_t event_mask) {
    // Essentially just a signal to stop logging data
    sem_acquire_blocking(&sem);
    state = END;
    snapshot();
    sem_acquire_blocking(&sem);
}

float get_altitude() {
    uint8_t reg = 0x01;
    uint8_t data[5];
    i2c_write_blocking(i2c_default, ALT_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, ALT_ADDR, data, 5, false);
    // Exactly how MPL3115A2 datasheet says to retrieve altitude
    float altitude = (float) ((int16_t) ((data[0] << 8) | data[1])) + (float) (data[2] >> 4) * 0.0625;
    // uint32_t temp_alt = (data[1] << 24) | (data[2] << 16) | (data[3] << 8);
    // float altitude = temp_alt / 65536.0f;
    return altitude;
}

/**
 * @brief Calculates the fitted Coeficient of Drag using the Surface Fit Model for the current rocket design.
 * @param velocity Velocity
 * @param altitude Altitude
 *
 * @return: Drag Coefficient (CD)
 */
float get_deploy_percent(float velocity, float altitude) {
    // Lookup table (Data from 'dragSurfFit.m' for Surface Fit Model Formula)
    float p00 = -8.498e+04;
    float p10 = 924.4;
    float p01 = 69.98;
    float p20 = -3.62;
    float p11 = -0.6196;
    float p02 = -0.01897;
    float p30 = 0.005983;
    float p21 = 0.001756;
    float p12 = 0.0001271;
    float p03 = 1.693e-06;
    float p40 = -3.451e-06;
    float p31 = -1.582e-06;
    float p22 = -2.004e-07;
    float p13 = -7.476e-09;


    /* MATLAB Code:
    * return p00 + p10 * V + p01 * H + p20 * V ** 2 + \
	* p11 * V * H + p02 * H ** 2 + p30 * V ** 3 + \
	* p21 * V ** 2 * H + p12 * V * H ** 2 + p03 * H ** 3 + \
	* p40 * V ** 4 + p31 * V ** 3 * H + p22 * V ** 2 * H ** 2 + \
	* p13 * V * H ** 3
    */

    return (p00 + p10 * velocity + p01 * altitude + p20 * std::pow(velocity, 2) + p11 * velocity * altitude + p02 * std::pow(altitude, 2)
    + p30 * std::pow(velocity, 3) + p21 * std::pow(velocity, 2) * altitude + p12 * velocity * std::pow(altitude, 2) + p03 * std::pow(altitude, 3)
    + p40 * std::pow(velocity, 4) + p31 * std::pow(velocity, 3) * altitude + p22 * std::pow(velocity, 2) * std::pow(altitude, 2) + p13 * velocity * std::pow(altitude, 3));


}

float get_velocity() {
    uint8_t reg = 0x07;
    uint8_t data[5];
    i2c_write_blocking(i2c_default, ALT_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c_default, ALT_ADDR, data, 5, false);
    float delta = (float) ((int16_t) ((data[0] << 8) | data[1])) + (float) (data[2] >> 4) * 0.0625;
    float vel = delta * DATA_RATE_HZ;
    return vel;
}

