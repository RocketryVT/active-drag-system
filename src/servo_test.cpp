#include <stdio.h>
#include <inttypes.h>
#include "pico/stdio.h"
#include "pwm.hpp"

#define MOSFET_PIN 1

PWM pwm;

int main() {
    stdio_init_all();
    // Initialize MOSFET
    gpio_init(MOSFET_PIN);
    gpio_set_dir(MOSFET_PIN, GPIO_OUT);
    gpio_put(MOSFET_PIN, 1);
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
