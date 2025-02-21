#include <cstdint>
#include <stdio.h>

#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "boards/pico.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"

#include "mct8316z.hpp"

#define MICRO_MOTOR_POS_SENS_SDA 14
#define MICRO_MOTOR_POS_SENS_SCL 15
#define MICRO_MOTOR_POS_ADDR 0x06

#define HEART_RATE_HZ 5


void heartbeat_core();
bool heartbeat_callback(repeating_timer_t *rt);

void process_cmd(char* buf, uint8_t len);
uint8_t cton(char c);

volatile uint8_t led_counter;
repeating_timer_t heartbeat_timer;
repeating_timer_t motor_timer;

mct8316z motor_driver(spi0);

bool mct8316z_update(repeating_timer_t* rt) {
    mct8316z* motor_driver = (mct8316z *) (rt->user_data);
    motor_driver->update();
    return true;
}

int main() {
    stdio_init_all();

    i2c_init(i2c1, 400000);


    gpio_set_function(MICRO_MOTOR_POS_SENS_SCL, GPIO_FUNC_I2C);
    gpio_set_function(MICRO_MOTOR_POS_SENS_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(MICRO_MOTOR_POS_SENS_SCL);
    gpio_pull_up(MICRO_MOTOR_POS_SENS_SDA);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);


    multicore_launch_core1(heartbeat_core);


    getchar();

    motor_driver.initialize();

    add_repeating_timer_us(-1000000 / MOTOR_UPDATE_HZ,  &mct8316z_update, ((void *) &motor_driver), &motor_timer);

    uint8_t src = 0x3;
    uint8_t ibuf[2] = {0x0, 0x0};

    i2c_write_blocking(i2c1, MICRO_MOTOR_POS_ADDR, &src, 1, true);
    i2c_read_blocking(i2c1, MICRO_MOTOR_POS_ADDR, ibuf, 2, false);
    uint16_t raw_angle = ((((uint16_t) ibuf[0]) << 6) | (((uint16_t) ibuf[1]) << 2));
    float angular_pos = (((float) raw_angle) / 16384.0f) * 360.0f;
    printf("Raw angle: %04X; float angle: %4.2f\n", raw_angle, angular_pos);

    char c;
    char buf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t idx = 0;
    printf(">\t");
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

    }
}

void heartbeat_core() {
    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);

    while (1) {
        tight_loop_contents();
    }
}

bool heartbeat_callback(repeating_timer_t *rt) {
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    bool led_status = sequence[led_counter];
    gpio_put(PICO_DEFAULT_LED_PIN, led_status);
    led_counter++;
    led_counter %= sequence_length;
    return true;
}

void process_cmd(char* buf, uint8_t len) {
    if (len > 0) {
        int8_t result = -1;
        switch (buf[0]) {
            case 'p': {
                if (len >= 4) {
                    uint8_t msb_duty = cton(buf[2]);
                    uint8_t lsb_duty = cton(buf[3]);
                    if (msb_duty <= 0xF && lsb_duty <= 0xF) {
                        uint8_t duty_cycle = (msb_duty << 4) | lsb_duty;
                        printf("\nOperating motor with a %d percent duty cycle\n", duty_cycle);
                        result = motor_driver.set_pwm(duty_cycle);
                        if (result < 0) printf("\nMotor is disabled! Enable motor with 'enable' to access this command!\n");
                        result = 0;
                    }
                    break;
                }
            }
            case 'v': {
                if (len >= 4) {
                    uint8_t msb_speed = cton(buf[2]);
                    uint8_t lsb_speed = cton(buf[3]);
                    if (msb_speed <= 0xF && lsb_speed <= 0xF) {
                        uint16_t speed_cmd_unsigned = ((((uint16_t) msb_speed) << 4) | ((uint16_t) lsb_speed)) * ((uint16_t) 100);
                        int16_t speed_cmd = *((int16_t *) &speed_cmd_unsigned);
                        printf("\nCommanding motor to %d RPM!\n", speed_cmd);
                        result = motor_driver.set_speed(speed_cmd);
                        if (result < 0) printf("\nMotor is disabled! Enable motor with 'enable' to access this command!\n");
                        result = 0;
                    }
                    break;
                }
            }
            case 's': {
                motor_driver.disable_motor();
                result = 0;
                break;
            }
            case 'e': {
                if (len >= 6) {
                    if (buf[1] == 'n' && buf[2] == 'a' && buf[3] == 'b' && buf[4] == 'l' && buf[5] == 'e') {
                        motor_driver.enable_motor();
                        result = 0;
                    }
                }
                break;
            }
            case 't': {
                uint8_t src = 0x3;
                uint8_t ibuf[2] = {0x0, 0x0};
                i2c_write_blocking(i2c1, MICRO_MOTOR_POS_ADDR, &src, 1, true);
                i2c_read_blocking(i2c1, MICRO_MOTOR_POS_ADDR, ibuf, 2, false);
                uint16_t raw_angle = ((((uint16_t) ibuf[0]) << 6) | (((uint16_t) ibuf[1]) << 2));
                float angular_pos = (((float) raw_angle) / 16384.0f) * 360.0f;
                printf("Raw angle: %04X; float angle: %4.2f\n", raw_angle, angular_pos);
                result = 0;
                break;
            }
            case 'y': {
                printf("\nMotor Speed: %d\n", motor_driver.get_speed());
                printf("\nMotor Speed Filtered: %d\n", motor_driver.get_speed_filtered());
                printf("\nMotor Speed Command: %d\n", motor_driver.get_speed_cmd());
                printf("\nMotor Speed Setpoint: %d\n", motor_driver.get_speed_setpoint());
                result = 0;
                break;
            }
            case 'f': {
                printf("Clearing motor driver faults!\n");
                motor_driver.clear_faults();
                result = 0;
                break;
            }
            case 'r': {
                int16_t old_setpoint = motor_driver.get_speed_setpoint();
                printf("\nReversing direction!\n");
                motor_driver.set_reverse_direction();
                printf("\nResetting setpoint to %d RPM!\n", old_setpoint);
                motor_driver.set_speed(old_setpoint);
                result = 0;
                break;
            }
            default:
                break;
        }
        if (result < 0) {
            printf("Invalid command, try again!\n");
        }
    }
}

/* Returns 255 if char not 0-9 or A-F */
uint8_t cton(char c) {
    uint8_t result = 255;
    if (c >= 'A' && c <= 'F') {
        result = (c - 'A') + 0xA;
    } else if (c >= 'a' && c <= 'f') {
        result = (c - 'a') + 0xA;
    } else if (c >= '0' && c <= '9') {
        result = (c - '0');
    }
    return result;
}

