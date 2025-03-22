#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/sem.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include <stdio.h>
#include <cstring>
#include <inttypes.h>
#include <math.h>

#include "pwm.hpp"
#include "altimeter.hpp"
#include "heartbeat.hpp"
#include "pico_logger.h"
#include "rp2040_micro.h"
#include "mid_imu.hpp"
#include "high_accel.hpp"
#include "magnetometer.hpp"
#define SERIAL_RATE_HZ 10

#define MOVING_AVG_MAX_SIZE 20
#define MAX_SCL 400000
#define DATA_RATE_HZ 200
#define LOOP_PERIOD (1.0f / DATA_RATE_HZ)

#define LOG_RATE_HZ 50

#define MOTOR_BURN_TIME 2600 // Burn time in milliseconds for L2200G

#define PACKET_SIZE 43
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

typedef struct {
    // 11 bytes General time and state data
    uint64_t time_us: 64;
    state_t state: 4;
    uint16_t temperature_chip: 12;
    uint8_t deploy_percent: 8;

    // 7.25 bytes MS5607 data
    uint32_t pressure: 18;
    int32_t altitude: 24;
    int16_t temperature_alt: 16;

    // 12 bytes IMU data
    int16_t ax : 16;
    int16_t ay : 16;
    int16_t az : 16;
    int16_t gx : 16;
    int16_t gy : 16;
    int16_t gz : 16;

    // 6.75 bytes MAG data
    int32_t mag_x: 18;         // 18 bits (1.125 bytes) == 27.125
    int32_t mag_y: 18;         // 18 bits (1.125 bytes) == 28.25
    int32_t mag_z: 18;         // 18 bits (1.125 bytes) == 29.375

    // 6 bytes High G Accel data
    int16_t high_g_x : 16;
    int16_t high_g_y : 16;
    int16_t high_g_z : 16;
} __attribute__((packed)) log_entry_t;

MidIMU mid(i2c_default);
HighAccel high(i2c_default);
Magnetometer mag(i2c_default);
PWM pwm;

int64_t pad_callback(alarm_id_t id, void* user_data);
int64_t boost_callback(alarm_id_t id, void* user_data);
int64_t apogee_callback(alarm_id_t id, void* user_data);
int64_t coast_callback(alarm_id_t id, void* user_data);
int64_t recovery_callback(alarm_id_t id, void* user_data);

bool timer_callback(repeating_timer_t *rt);

bool logging_buffer_callback(repeating_timer_t *rt);
bool logging_flash_callback(repeating_timer_t *rt);

void process_cmd(char* buf, uint8_t len);
bool serial_callback(repeating_timer_t *rt);

void logging_core();

void populate_log_entry();
void print_log_entry(const uint8_t* entry);

semaphore_t sem;

volatile int32_t altitude = 0;
volatile int32_t previous_altitude = 0;
volatile int32_t velocity = 0;
volatile state_t state = PAD;
volatile int32_t threshold_altitude = 30;
volatile float threshold_velocity = 30.0f;
volatile uint8_t deployment_percent = 0;

volatile bool serial_data_output = false;

repeating_timer_t data_timer;
repeating_timer_t log_timer;
repeating_timer_t serial_timer;

int32_t ground_altitude = 0;


volatile int32_t moving_average[MOVING_AVG_MAX_SIZE];
volatile size_t moving_average_offset = 0;
volatile size_t moving_average_size = 0;
volatile int32_t moving_average_sum = 0;

altimeter altimeter(i2c_default);

Logger logger(PACKET_SIZE, LOG_BASE_ADDR, &print_log_entry);
log_entry_t log_entry;

int main() {
    stdio_init_all();

    adc_init();
    adc_set_temp_sensor_enabled(true);

    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    alarm_pool_init_default();

    altimeter.initialize();
    sleep_ms(100);
    logger.initialize(true);
    logger.initialize_circular_buffer(PAD_BUFFER_SIZE);

    sleep_ms(100);

    altimeter.ms5607_start_sample();
    sleep_ms(100);
    ground_altitude = altimeter.get_altitude();
    altimeter.set_threshold_altitude(ground_altitude + (threshold_altitude * ALTITUDE_SCALE), &pad_callback);

    high.initialize();
    mag.initialize();
    mid.initialize();

    pwm.init();

    // Initialize MOSFET
    gpio_init(MICRO_DEFAULT_SERVO_ENABLE);
    gpio_set_dir(MICRO_DEFAULT_SERVO_ENABLE, GPIO_OUT);
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);

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

    altimeter.ms5607_start_sample();
    high.getData();
    mag.getData();
    mid.getData();

    if (moving_average_size == MOVING_AVG_MAX_SIZE) {
        moving_average_sum -= moving_average[moving_average_offset];
    } else {
        moving_average_size++;
    }

    moving_average[moving_average_offset] = altimeter.get_altitude();
    moving_average_sum += moving_average[moving_average_offset];
    moving_average_offset = (moving_average_offset + 1) % MOVING_AVG_MAX_SIZE;

    velocity = (altitude - previous_altitude) * DATA_RATE_HZ;
    previous_altitude = altitude;
    altitude = moving_average_sum / moving_average_size;

    deployment_percent = 80;

    switch(state) {
        case PAD:
            gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case BOOST:
            gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case COAST:
            gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
            pwm.set_servo_percent(deployment_percent);
            break;
        case APOGEE:
            gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case RECOVERY:
            gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
        case END:
            gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 0);
            pwm.set_servo_percent(0);
            deployment_percent = 0;
            break;
    }
    sem_release(&sem);
    return true;
}

int64_t pad_callback(alarm_id_t id, void* user_data) {
    sem_acquire_blocking(&sem);
    altimeter.clear_threshold_altitude();
    state = BOOST;
    sem_release(&sem);
    // start motor burn timer with boost transition function as callback
    add_alarm_in_ms(MOTOR_BURN_TIME, &boost_callback, NULL, false);
    return 0;
}

int64_t boost_callback(alarm_id_t id, void* user_data) {
    sem_acquire_blocking(&sem);
    state = COAST;
    populate_log_entry();
    logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), true);
    sem_release(&sem);
    add_alarm_in_ms(1000, &coast_callback, NULL, false);
    return 0;
}

int64_t coast_callback(alarm_id_t id, void* user_data) {
    if (velocity <= 0) {
        sem_acquire_blocking(&sem);
        state = APOGEE;
        populate_log_entry();
        logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), false);
        sem_release(&sem);
        add_alarm_in_ms(1, &apogee_callback, NULL, false);
    } else {
        add_alarm_in_ms(50, &coast_callback, NULL, false);
    }
    return 0;
}

int64_t apogee_callback(alarm_id_t id, void* user_data) {
    state = RECOVERY;
    // Set altimeter interrupt to occur for when rocket touches back to the ground
    altimeter.set_threshold_altitude((ground_altitude + 100), &recovery_callback);

    sem_acquire_blocking(&sem);
    populate_log_entry();
    logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), true);
    sem_release(&sem);
    return 0;
}

int64_t recovery_callback(alarm_id_t id, void* user_data) {
    // Essentially just a signal to stop logging data
    sem_acquire_blocking(&sem);
    state = END;
    populate_log_entry();
    logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), true);
    sem_acquire_blocking(&sem);
    return 0;
}


// LOGGING THREAD RELATED FUNCTIONS AND CALLBACKS
//===============================================================================

void logging_core() {
    add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_buffer_callback, NULL, &log_timer);

    add_repeating_timer_us(-1000000 / SERIAL_RATE_HZ,  &serial_callback, NULL, &serial_timer);

    heartbeat_initialize(PICO_DEFAULT_LED_PIN);

    while (1) {
        tight_loop_contents();
    }
}

void populate_log_entry() {
    absolute_time_t now = get_absolute_time();
    log_entry.time_us = to_us_since_boot(now);

    adc_select_input(4);
    log_entry.temperature_chip = adc_read();
    log_entry.state = state;
    log_entry.deploy_percent = deployment_percent;
    log_entry.pressure = altimeter.get_pressure();
    log_entry.altitude = altimeter.get_altitude();
    log_entry.temperature_alt = altimeter.get_temperature();

    log_entry.ax = mid.get_ax();
    log_entry.ay = mid.get_ay();
    log_entry.az = mid.get_az();
    log_entry.gx = mid.get_gx();
    log_entry.gy = mid.get_gy();
    log_entry.gz = mid.get_gz();

    log_entry.mag_x = mag.get_ax();
    log_entry.mag_y = mag.get_ay();
    log_entry.mag_z = mag.get_az();

    log_entry.high_g_x = high.get_ax();
    log_entry.high_g_y = high.get_ay();
    log_entry.high_g_z = high.get_az();

}

bool logging_buffer_callback(repeating_timer_t *rt) {
    sem_acquire_blocking(&sem);
    populate_log_entry();
    sem_release(&sem);

    logger.write_circular_buffer(reinterpret_cast<const uint8_t *>(&log_entry));

    if (state != PAD) {
        sem_acquire_blocking(&sem);
        logger.flush_circular_buffer(true);
        sem_release(&sem);
        cancel_repeating_timer(&log_timer);
        add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_flash_callback, NULL, &log_timer);
    }
    return true;
}

bool logging_flash_callback(repeating_timer_t *rt) {
    sem_acquire_blocking(&sem);
    populate_log_entry();
    logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), false);
    sem_release(&sem);
    if (state == END) {
        logger.flush_buffer();
        cancel_repeating_timer(&log_timer);
    }
    return true;
}

void print_log_entry(const uint8_t* entry) {
    static bool first_call = true;

    if (first_call) {
        printf("time_us,state,temperature_chip_celsius,deployment_percent,pressure_mb,altitude_m,temperature_alt_celsius,mid_imu_ax,mid_imu_ay,mid_imu_az,mid_imu_gx,mid_imu_gy,mid_imu_gz,mag_x,mag_y,mag_z,high_g_x,high_g_y,high_g_z\r\n");
        first_call = false;
    }

    const log_entry_t* packet = reinterpret_cast<const log_entry_t *>(entry);

    printf("%" PRIu64 ",", packet->time_us);
    state_t state = (state_t) packet->state;
    switch (state) {
        case PAD:
            printf("PAD,");
            break;
        case BOOST:
            printf("BOOST,");
            break;
        case COAST:
            printf("COAST,");
            break;
        case APOGEE:
            printf("APOGEE,");
            break;
        case RECOVERY:
            printf("RECOVERY,");
            break;
        case END:
            printf("END,");
            break;
    }
    const float conversionFactor = 3.3f / (1 << 12);
    float tempC = 27.0f - (((float)(packet->temperature_chip) * conversionFactor) - 0.706f) / 0.001721f;
    printf("%4.2f,", tempC);
    printf("%d,", packet->deploy_percent);
    printf("%4.2f,", ((float) packet->pressure) / PRESSURE_SCALE_F);
    printf("%4.2f,", ((float) packet->altitude) / ALTITUDE_SCALE_F);
    printf("%4.2f,", ((float) packet->temperature_alt) / TEMPERATURE_SCALE_F);

    printf("%4.2f,", mid.scale_accel(packet->ax));
    printf("%4.2f,", mid.scale_accel(packet->ay));
    printf("%4.2f,", mid.scale_accel(packet->az));
    
    printf("%4.2f,", mid.scale_gyro(packet->gx));
    printf("%4.2f,", mid.scale_gyro(packet->gy));
    printf("%4.2f,", mid.scale_gyro(packet->gz));
    
    printf("%4.2f,", mag.scale(packet->mag_x));
    printf("%4.2f,", mag.scale(packet->mag_y));
    printf("%4.2f,", mag.scale(packet->mag_z));
    
    printf("%4.2f,", high.scale(packet->high_g_x));
    printf("%4.2f,", high.scale(packet->high_g_y));
    printf("%4.2f", high.scale(packet->high_g_z));
    printf("\r\n");
}

bool serial_callback(repeating_timer_t *rt) {
    static char c;
    static char buf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    static uint8_t idx = 0;

    c = getchar_timeout_us(0);
    if (c != 255) {
        if (idx < 16) {
            if (c == 0x08 && idx > 0) { /* backspace */
                idx--;
                buf[idx] = 0;
                printf("%c", 0x7f);
            }
            else if (c == 0x0D) { /* carriage return (enter) */
                process_cmd(buf, idx);
                for (uint8_t i = 16; i < 16; i++) {
                    buf[i] = 0;
                }
                idx = 0;
                printf("\n>\t");
            }
            else {
                buf[idx] = c;
                idx++;
            }
            printf("%c", c);
        } else {
            printf("Overflow! press enter to reset buffer!\n");
            if (c == 0x0D) {
                for (uint8_t i = 16; i < 16; i++) {
                    buf[i] = 0;
                }
                idx = 0;
                printf("\n>\t");
            }
        }
    }

    if (serial_data_output) {
     //   switch (state) {
     //       case PAD:
     //           printf("PAD");
     //           break;
     //       case BOOST:
     //           printf("BOOST");
     //           break;
     //       case COAST:
     //           printf("COAST");
     //           break;
     //       case APOGEE:
     //           printf("APOGEE");
     //           break;
     //       case RECOVERY:
     //           printf("RECOVERY");
     //           break;
     //       case END:
     //           printf("END");
     //           break;
     //   }
     //   printf(": Altitude: %4.2f, Velocity: %4.2f\n", ((float) altitude) / ALTITUDE_SCALE_F, ((float) velocity) / ALTITUDE_SCALE_F);
     print_log_entry(reinterpret_cast<const uint8_t *>(&log_entry));
    }

    return true;
}

void process_cmd(char* buf, uint8_t len) {
    if (len > 0) {
        int8_t result = -1;
        switch (buf[0]) {
            case 'r': {
                if (len >= 4) {
                    if (buf[1] == 'e' && buf[2] == 'a' && buf[3] == 'd') {
                        printf("\nReading memory!\n");
                        uint32_t status = save_and_disable_interrupts();
                        logger.read_memory();
                        restore_interrupts(status);
                        result = 0;
                    }
            }
            case 'e': {
                if (len >= 5) {
                    if (buf[1] == 'r' && buf[2] == 'a' && buf[3] == 's' && buf[4] == 'e') {
                        printf("Are you sure you want to erase all log memory? ");
                        char c = getchar();
                        if (c == 'y' or c == 'Y') {
                            printf("\nErasing memory!\n");
                            logger.erase_memory();
                        } else {
                            printf("\nMemory will NOT be erased!\n");
                        }
                        result = 0;
                    }
                }
            }
                break;
            case 't': {
                if (len >= 6) {
                    if (buf[1] == 'o' && buf[2] == 'g' && buf[3] == 'g' && buf[4] == 'l' && buf[5] == 'e') {
                        serial_data_output = !serial_data_output;
                        result = 0;
                    }
                }
            }
            case 'i':
                    logger.initialize(true);
                    result = 0;
                break;
            default:
                break;
        }
        if (result < 0) {
            printf("Invalid command, try again!\n");
        }
    
        }
    }
}
