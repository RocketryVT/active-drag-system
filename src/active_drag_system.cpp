#include "cyw43_configport.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/sem.h"
#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "pico/cyw43_arch.h"
#include <inttypes.h>
#include <math.h>

#include "pwm.hpp"
#include "imu.hpp"
#include "altimeter.hpp"
#include "kalman_filter.hpp"
#include "spi_flash.h"

#define MPL3115A2_ADDR 0x60

#define BNO055_ADDR 0x28
#define BNO055_ID 0xA0

#define MAX_SCL 400000
#define DATA_RATE_HZ 100
#define LOOP_PERIOD (1.0f / DATA_RATE_HZ)
#define INT1_PIN 6 // INT1 PIN on MPL3115A2 connected to GPIO PIN 9 (GP6)
#define MOSFET_PIN 26 // MOSFET PIN connected to GPIO PIN 31 (GP26)

#define LOG_RATE_HZ 8
#define HEART_RATE_HZ 5

#define MOTOR_BURN_TIME 3900 // Burn time in milliseconds for M2500T

#define PAD_SECONDS 8
#define PAD_BUFFER_SIZE (PACKET_SIZE * LOG_RATE_HZ * PAD_SECONDS)

typedef enum {
    PAD = 0,
    BOOST,
    COAST,
    APOGEE,
    RECOVERY,
    END
} state_t;

PWM pwm;
kalman_filter *kf;
VectorXf control(1);
VectorXf measurement(1);
VectorXf res(2);

void pad_callback(uint gpio, uint32_t event_mask);
int64_t boost_callback(alarm_id_t id, void* user_data);
int64_t apogee_callback(alarm_id_t id, void* user_data);
int64_t coast_callback(alarm_id_t id, void* user_data);
void recovery_callback(uint gpio, uint32_t event_mask);
float get_deploy_percent(float velocity, float altitude);

bool timer_callback(repeating_timer_t *rt);

void populate_entry();
bool logging_buffer_callback(repeating_timer_t *rt);
bool logging_flash_callback(repeating_timer_t *rt);
bool heartbeat_callback(repeating_timer_t *rt);
void logging_core();

semaphore_t sem;

volatile float altitude = 0.0f;
volatile float velocity = 0.0f;
volatile state_t state = PAD;
volatile float threshold_altitude = 30.0f;
volatile float threshold_velocity = 30.0f;
volatile uint8_t deployment_percent = 0;
volatile uint8_t led_counter;
volatile uint32_t pad_buffer_offset = 0;

Eigen::Vector3f linear_acceleration;
Eigen::Vector4f quaternion;
Eigen::Vector3f euler_angles;

volatile calibration_status_t calib_status;

repeating_timer_t data_timer;
repeating_timer_t log_timer;
repeating_timer_t heartbeat_timer;

float ground_altitude = 0.0f;

altimeter altimeter(i2c_default, MPL3115A2_ADDR);
imu imu(i2c_default, BNO055_ADDR, BNO055_ID, NDOF);

uint8_t *altimeter_buffer;
uint8_t *acceleration_buffer;
uint8_t *quaternion_buffer;

uint8_t entry_buffer[PACKET_SIZE];

uint8_t *pad_buffer;

int main() {
    adc_init();
    adc_set_temp_sensor_enabled(true);

    cyw43_arch_init();

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

    gpio_init(INT1_PIN);
    gpio_pull_up(INT1_PIN);
    
    alarm_pool_init_default();

    altimeter.initialize(30.0f, INT1_PIN, &pad_callback);

    imu.initialize();
    imu.linear_acceleration(linear_acceleration);
    imu.quaternion(quaternion);
    imu.quaternion_euler(euler_angles, quaternion);

    pwm.init();

    // Initialize MOSFET
    gpio_init(MOSFET_PIN);
    gpio_set_dir(MOSFET_PIN, GPIO_OUT);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    pad_buffer = (uint8_t*)malloc(PAD_BUFFER_SIZE);

    // Initialize Kalman Filter
    kf = new kalman_filter(2, 1, 1, 0.01);
    VectorXf state_vec(2);
    MatrixXf state_cov(2, 2);
    state_vec << altimeter.get_altitude_converted(), linear_acceleration.z();
    state_cov << 0.018, 0.0, 0.0, 0.0005;
    kf->state_initialize(state_vec, state_cov);
    ground_altitude = altimeter.get_altitude_converted();

    altimeter.expose_buffer(&altimeter_buffer);
    imu.expose_acceleration_buffer(&acceleration_buffer);
    imu.expose_quaternion_buffer(&quaternion_buffer);

    sem_init(&sem, 1, 1);

    add_repeating_timer_us(-1000000 / DATA_RATE_HZ,  &timer_callback, NULL, &data_timer);

    multicore_launch_core1(logging_core);

    while (1) {
        tight_loop_contents();
    }
}

// PRIMARY THREAD RELATED FUNCTIONS AND CALLBACKS
//===============================================================================

bool timer_callback(repeating_timer_t *rt) {
    sem_acquire_blocking(&sem);
    imu.linear_acceleration(linear_acceleration);
    imu.quaternion(quaternion);

    control(0) = linear_acceleration.z();
    measurement(0) = altimeter.get_altitude_converted();
    res = kf->run(control, measurement, 0.01f);
    altitude = res(0);
    velocity = res(1);


    deployment_percent = (uint8_t)(std::min(std::max(30.0f, get_deploy_percent(velocity, (altitude - ground_altitude))), 100.0f));

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
    sem_release(&sem);
    return true;
}

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
    sem_acquire_blocking(&sem);
    altimeter.unset_threshold_altitude(INT1_PIN);
    state = BOOST;
    sem_release(&sem);
    // start motor burn timer with boost transition function as callback
    add_alarm_in_ms(MOTOR_BURN_TIME, &boost_callback, NULL, false);
}

int64_t boost_callback(alarm_id_t id, void* user_data) {
    // Configure accelerometer and/or altimeter to generate interrupt
    // for when velocity is negative with this function as callback to
    // transition to APOGEE
    sem_acquire_blocking(&sem);
    state = COAST;
    populate_entry();
    write_entry(entry_buffer);
    sem_release(&sem);
    add_alarm_in_ms(1000, &coast_callback, NULL, false);
    return 0;
}

int64_t coast_callback(alarm_id_t id, void* user_data) {
    // Want to somehow immediately transition to RECOVERY from APOGEE (extremely short timer?)
    if (velocity <= 0.0f) {
        sem_acquire_blocking(&sem);
        state = APOGEE;
        populate_entry();
        write_entry(entry_buffer);
        sem_release(&sem);
        add_alarm_in_ms(1, &apogee_callback, NULL, false);
    } else {
        add_alarm_in_ms(250, &coast_callback, NULL, false);
    }
    return 0;
}

int64_t apogee_callback(alarm_id_t id, void* user_data) {
    state = RECOVERY;
    // Set altimeter interrupt to occur for when rocket touches back to the ground
    altimeter.set_threshold_altitude((ground_altitude + 10.0f), INT1_PIN, &recovery_callback);

    sem_acquire_blocking(&sem);
    populate_entry();
    write_entry(entry_buffer);
    sem_release(&sem);
    return 0;
}

void recovery_callback(uint gpio, uint32_t event_mask) {
    // Essentially just a signal to stop logging data
    sem_acquire_blocking(&sem);
    state = END;
    populate_entry();
    write_entry(entry_buffer);
    sem_acquire_blocking(&sem);
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

// LOGGING THREAD RELATED FUNCTIONS AND CALLBACKS
//===============================================================================

void logging_core() {
    add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_buffer_callback, NULL, &log_timer);
    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);

    while (1) {
        tight_loop_contents();
    }
}

void populate_entry() {
    absolute_time_t now = get_absolute_time();
    uint64_t now_us = to_us_since_boot(now);
    uint32_t vel_bits = *((uint32_t *)&velocity);
    entry_buffer[0] = now_us >> 56;
    entry_buffer[1] = now_us >> 48;
    entry_buffer[2] = now_us >> 40;
    entry_buffer[3] = now_us >> 32;
    entry_buffer[4] = now_us >> 24;
    entry_buffer[5] = now_us >> 16;
    entry_buffer[6] = now_us >> 8;
    entry_buffer[7] = now_us;

    adc_select_input(4);
    uint16_t temperature = adc_read();
    entry_buffer[8] = ((*(uint8_t *)(&state)) << 4) | (temperature >> 8);
    entry_buffer[9] = temperature;

    entry_buffer[10] = deployment_percent;
    entry_buffer[11] = altimeter_buffer[0];
    entry_buffer[12] = altimeter_buffer[1];
    entry_buffer[13] = altimeter_buffer[2];
    entry_buffer[14] = vel_bits >> 24;
    entry_buffer[15] = vel_bits >> 16;
    entry_buffer[16] = vel_bits >> 8;
    entry_buffer[17] = vel_bits;
    entry_buffer[18] = acceleration_buffer[0];
    entry_buffer[19] = acceleration_buffer[1];
    entry_buffer[20] = acceleration_buffer[2];
    entry_buffer[21] = acceleration_buffer[3];
    entry_buffer[22] = acceleration_buffer[4];
    entry_buffer[23] = acceleration_buffer[5];
    entry_buffer[24] = quaternion_buffer[0];
    entry_buffer[25] = quaternion_buffer[1];
    entry_buffer[26] = quaternion_buffer[2];
    entry_buffer[27] = quaternion_buffer[3];
    entry_buffer[28] = quaternion_buffer[4];
    entry_buffer[29] = quaternion_buffer[5];
    entry_buffer[30] = quaternion_buffer[6];
    entry_buffer[31] = quaternion_buffer[7];
}

bool logging_buffer_callback(repeating_timer_t *rt) {
    sem_acquire_blocking(&sem);
    populate_entry();
    sem_release(&sem);
    for (uint32_t i = 0; i < PACKET_SIZE; i++) {
        pad_buffer[i + pad_buffer_offset] = entry_buffer[i];
    }
    pad_buffer_offset += PACKET_SIZE;
    pad_buffer_offset %= PAD_BUFFER_SIZE;

    if (state != PAD) {
        uint32_t idx = ((pad_buffer_offset + PACKET_SIZE) % PAD_BUFFER_SIZE);
        sem_acquire_blocking(&sem);
        do {
            write_entry(pad_buffer + idx);
            idx += PACKET_SIZE;
            idx %= PAD_BUFFER_SIZE;
        } while (idx != pad_buffer_offset);
        sem_release(&sem);
        cancel_repeating_timer(&log_timer);
        free(pad_buffer);
        add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_flash_callback, NULL, &log_timer);
    }
    return true;
}

bool logging_flash_callback(repeating_timer_t *rt) {
    sem_acquire_blocking(&sem);
    populate_entry();
    write_entry(entry_buffer);
    sem_release(&sem);
    if (state == END) {
        cancel_repeating_timer(&log_timer);
    }
    return true;
}

bool heartbeat_callback(repeating_timer_t *rt) {
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    bool led_status = sequence[led_counter];
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_status);
    led_counter++;
    led_counter %= sequence_length;
    return true;
}

