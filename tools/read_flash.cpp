#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/multicore.h"
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "pico_logger.h"
#include "hardware/adc.h"

#include "mid_imu.hpp"
#include "high_accel.hpp"
#include "magnetometer.hpp"
#include "altimeter.hpp"

#include "heartbeat.hpp"
#include "log_format.hpp"

#define MAX_SCL 400000
#define SERIAL_RATE_HZ 10
#define MOVING_AVG_MAX_SIZE 20

const char* read_cmd            = "read";
const char* erase_cmd           = "erase";
const char* read_circular_cmd   = "rcircle";
const char* toggle_circular_cmd = "tcircle";
const char* flush_circular_cmd  = "fcircle";
const char* start_log_cmd       = "logstart";
const char* stop_log_cmd        = "logstop";
const char* toggle_debug_cmd    = "tdebug";

const uint32_t num_cmds = 8;

const char* cmds[num_cmds] = {read_cmd, erase_cmd, read_circular_cmd, toggle_circular_cmd, flush_circular_cmd, start_log_cmd, stop_log_cmd, toggle_debug_cmd};
const size_t cmd_lens[num_cmds] = {4, 5, 7, 7, 7, 8, 7, 6};

extern MidIMU* log_mid;
extern HighAccel* log_high;
extern Magnetometer* log_mag;

MidIMU mid(i2c_default);
HighAccel high(i2c_default);
Magnetometer mag(i2c_default);
altimeter altimeter(i2c_default);

void core1_entry();
bool serial_callback(repeating_timer_t *rt);
bool logging_buffer_callback(repeating_timer_t *rt);
bool logging_flash_callback(repeating_timer_t *rt);
void process_cmd(char* buf, uint8_t len);
void update_data();
void populate_log_entry();

repeating_timer_t serial_timer;
repeating_timer_t log_timer;

volatile bool serial_debug = false;
volatile bool logging = false;
volatile bool use_circular_buffer = false;

log_entry_t log_entry;

volatile int32_t altitude = 0;
volatile int32_t previous_altitude = 0;
volatile int32_t velocity = 0;
volatile state_t state = PAD;
volatile uint8_t deployment_percent = 0;

volatile int32_t moving_average[MOVING_AVG_MAX_SIZE];
volatile size_t moving_average_offset = 0;
volatile size_t moving_average_size = 0;
volatile int32_t moving_average_sum = 0;

char c;
char buf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t idx = 0;

Logger logger(PACKET_SIZE, LOG_BASE_ADDR, &print_log_entry);

int main() {
    stdio_init_all();
    log_mid = &mid;
    log_mag = &mag;
    log_high = &high;

    adc_init();
    adc_set_temp_sensor_enabled(true);

    i2c_init(i2c_default, MAX_SCL);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    alarm_pool_init_default();

    altimeter.initialize();
    high.initialize();
    mag.initialize();
    mid.initialize();

    logger.initialize(false);

    multicore_launch_core1(core1_entry);

//    add_repeating_timer_us(-1000000 / SERIAL_RATE_HZ,  &serial_callback, NULL, &serial_timer);

    while (1) {

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
                else if (c >= 32 && c <= 126) {
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
//        tight_loop_contents();
    }
}

void core1_entry() {
    heartbeat_initialize(PICO_DEFAULT_LED_PIN);

    while (1) {
        tight_loop_contents();
    }
}

bool serial_callback(repeating_timer_t *rt) {
    c = getchar_timeout_us(1000);
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

    return true;
}

void process_cmd(char* buf, uint8_t len) {
    if (len > 0) {
        const char** cmd = NULL;
        size_t cmp_len = 0;
        for (int i = 0; i < num_cmds; i++) {
            cmp_len = (len < cmd_lens[i]) ? len : cmd_lens[i];
            int result = strncmp(cmds[i], buf, cmp_len);
            if (result == 0) {
                cmd = (cmds + i);
                break;
            }
        }
        if (cmd != NULL) {
           if (cmd == (&read_cmd)) {
               if (!logging) {
                   logger.read_memory();
               } else {
                   printf("Disable logging before reading!\r\n");
               }
           }
           else if (cmd == (&erase_cmd)) {
               if (!logging) {
                   logger.erase_memory();
               } else {
                   printf("Disable logging before reading!\r\n");
               }
           }
           else if (cmd == (&read_circular_cmd)) {
               if (!logging) {
                   logger.read_circular_buffer();
               } else {
                   printf("Disable logging before reading!\r\n");
               }
           }
           else if (cmd == (&toggle_circular_cmd)) {
               if (!use_circular_buffer) {
                   logger.initialize_circular_buffer(PAD_BUFFER_SIZE);
                   use_circular_buffer = true;
               }
           }
           else if (cmd == (&flush_circular_cmd)) {
               if (logging) {
                   cancel_repeating_timer(&log_timer);
                   logger.flush_circular_buffer(true);
                   add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_flash_callback, NULL, &log_timer);
               } else {
                   logger.flush_circular_buffer(true);
               }
           }
           else if (cmd == (&start_log_cmd)) {
               if (!logging) {
                   if (use_circular_buffer) {
                       add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_buffer_callback, NULL, &log_timer);
                   } else {
                       add_repeating_timer_us(-1000000 / LOG_RATE_HZ,  &logging_flash_callback, NULL, &log_timer);
                   }
                   logging = true;
               } else {
                   printf("Already logging!\r\n");
               }
           }
           else if (cmd == (&stop_log_cmd)) {
               if (logging) {
                   cancel_repeating_timer(&log_timer);
                   logging = false;
               } else {
                   printf("Not logging!\r\n");
               }
           }
           else if (cmd == (&toggle_debug_cmd)) {
               serial_debug = !serial_debug;
           }
        } else {
            printf("Invalid command, try again!\n");
        }
    }
}

bool logging_buffer_callback(repeating_timer_t *rt) {
    update_data();
    populate_log_entry();

    if (serial_debug) {
        print_log_entry(reinterpret_cast<const uint8_t *>(&log_entry));
    }

    logger.write_circular_buffer(reinterpret_cast<const uint8_t *>(&log_entry));

    return true;
}

bool logging_flash_callback(repeating_timer_t *rt) {
    update_data();
    populate_log_entry();

    if (serial_debug) {
        print_log_entry(reinterpret_cast<const uint8_t *>(&log_entry));
    }

    logger.write_memory(reinterpret_cast<const uint8_t *>(&log_entry), false);
    return true;
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

void update_data() {
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

    velocity = (altitude - previous_altitude) * LOG_RATE_HZ;
    previous_altitude = altitude;
    altitude = moving_average_sum / moving_average_size;

    deployment_percent = 80;
}
