#include <stdio.h>
#include <inttypes.h>
#include "pico/stdio.h"
#include "pwm.hpp"

PWM pwm;

int main() {
    stdio_init_all();
    // Initialize MOSFET
    gpio_init(MICRO_DEFAULT_SERVO_ENABLE);
    gpio_set_dir(MICRO_DEFAULT_SERVO_ENABLE, GPIO_OUT);
    gpio_put(MICRO_DEFAULT_SERVO_ENABLE, 1);
    pwm.init();
    uint8_t duty_cycle = 13;
    while (1) {
        getchar();
        if (duty_cycle == 2) {
            duty_cycle = 13;
        }
        pwm.set_duty_cycle(duty_cycle);
        printf("Currenty Duty Cycle: %" PRIu8 "\n", duty_cycle);
        duty_cycle--;
    }
}
