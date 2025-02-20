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

mct8316z motor_driver(spi0);

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
                if (len >= 7) {
                    uint8_t msb_freq = cton(buf[2]);
                    uint8_t lsb_freq = cton(buf[3]);

                    uint8_t msb_duty = cton(buf[5]);
                    uint8_t lsb_duty = cton(buf[6]);
                    if (msb_freq <= 0xF && lsb_freq <= 0xF && msb_duty <= 0xF && lsb_duty <= 0xF) {
                        uint32_t frequency = (msb_freq << 4) | lsb_freq;
                        uint32_t duty_cycle = (msb_duty << 4) | lsb_duty;
                        if (frequency > 200) frequency = 200;
                        if (duty_cycle > 100) duty_cycle = 100;

                        if (motor_driver.is_motor_enabled()) {
                            printf("\nOperating motor at %d kHz with a %d percent duty cycle\n", frequency, duty_cycle);
                            frequency *= 1000;
                            duty_cycle = duty_cycle * (UINT16_MAX / 100);
                            motor_driver.set_pwm(frequency, duty_cycle);
                        } else {
                            printf("\nMotor is disabled! Enable motor with 'enable' to access this command!\n");
                        }
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
                break;
            }
            case 'y': {
                printf("\nMotor Speed: %d\n", motor_driver.get_speed());
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

