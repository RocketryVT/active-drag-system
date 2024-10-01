#include <cstdint>
#include <inttypes.h>
#include <stdio.h>
#include "pico/platform.h"
#include "pico/stdio.h"
#include "pico/types.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#define PHASE_A_HIGH 0
#define PHASE_B_HIGH 1
#define PHASE_C_HIGH 2
#define PHASE_A_LOW 3
#define PHASE_B_LOW 4
#define PHASE_C_LOW 5

#define HALL_A 15
#define HALL_B 14
#define HALL_C 13

volatile bool hall_a_triggered = false;
volatile bool hall_b_triggered = false;
volatile bool hall_c_triggered = false;
volatile uint8_t state = 0;

repeating_timer_t data_timer;

void hall_a_callback(uint gpio, uint32_t event_mask) {
    hall_a_triggered = true;
}

void hall_b_callback(uint gpio, uint32_t event_mask) {
    hall_b_triggered = true;
}

void hall_c_callback(uint gpio, uint32_t event_mask) {
    hall_c_triggered = true;
}

bool timer_callback(repeating_timer_t *rt) {
    if (hall_a_triggered || hall_b_triggered || hall_c_triggered) {
        state++;
        hall_a_triggered = false;
        hall_b_triggered = false;
        hall_c_triggered = false;
    }

    if (state > 5) {
        state = 0;
    }

    printf("state: %" PRIu8 "\r", state);

    switch (state) {
        case 0:
            gpio_put(PHASE_C_HIGH, 0);
            gpio_put(PHASE_A_LOW, 0);
            gpio_put(PHASE_B_HIGH, 0);
            gpio_put(PHASE_C_LOW, 0);

            gpio_put(PHASE_A_HIGH, 1);
            gpio_put(PHASE_B_LOW, 1);
            break;
        case 1:
            gpio_put(PHASE_C_HIGH, 0);
            gpio_put(PHASE_A_LOW, 0);
            gpio_put(PHASE_B_HIGH, 0);
            gpio_put(PHASE_B_LOW, 0);

            gpio_put(PHASE_A_HIGH, 1);
            gpio_put(PHASE_C_LOW, 1);
            break;
        case 2:
            gpio_put(PHASE_C_HIGH, 0);
            gpio_put(PHASE_A_LOW, 0);
            gpio_put(PHASE_A_HIGH, 0);
            gpio_put(PHASE_B_LOW, 0);

            gpio_put(PHASE_B_HIGH, 1);
            gpio_put(PHASE_C_LOW, 1);
            break;
        case 3:
            gpio_put(PHASE_C_HIGH, 0);
            gpio_put(PHASE_C_LOW, 0);
            gpio_put(PHASE_A_HIGH, 0);
            gpio_put(PHASE_B_LOW, 0);

            gpio_put(PHASE_B_HIGH, 1);
            gpio_put(PHASE_A_LOW, 1);
            break;
        case 4:
            gpio_put(PHASE_B_HIGH, 0);
            gpio_put(PHASE_C_LOW, 0);
            gpio_put(PHASE_A_HIGH, 0);
            gpio_put(PHASE_B_LOW, 0);

            gpio_put(PHASE_C_HIGH, 1);
            gpio_put(PHASE_A_LOW, 1);
            break;
        case 5:
            gpio_put(PHASE_B_HIGH, 0);
            gpio_put(PHASE_C_LOW, 0);
            gpio_put(PHASE_A_HIGH, 0);
            gpio_put(PHASE_A_LOW, 0);

            gpio_put(PHASE_C_HIGH, 1);
            gpio_put(PHASE_B_LOW, 1);
            break;
        default:
            break;
    }
    return 0;
}

int main() {
    stdio_init_all();

    getchar();

    gpio_init(PHASE_A_HIGH);
    gpio_init(PHASE_B_HIGH);
    gpio_init(PHASE_C_HIGH);

    gpio_init(PHASE_A_LOW);
    gpio_init(PHASE_B_LOW);
    gpio_init(PHASE_C_LOW);

    gpio_init(HALL_A);
    gpio_init(HALL_B);
    gpio_init(HALL_C);

    gpio_set_dir(PHASE_A_HIGH, GPIO_OUT);
    gpio_set_dir(PHASE_B_HIGH, GPIO_OUT);
    gpio_set_dir(PHASE_C_HIGH, GPIO_OUT);
    gpio_set_dir(PHASE_A_LOW, GPIO_OUT);
    gpio_set_dir(PHASE_B_LOW, GPIO_OUT);
    gpio_set_dir(PHASE_C_LOW, GPIO_OUT);

    gpio_put(PHASE_A_HIGH, 0);
    gpio_put(PHASE_B_HIGH, 0);
    gpio_put(PHASE_C_HIGH, 0);
    gpio_put(PHASE_A_LOW, 0);
    gpio_put(PHASE_B_LOW, 0);
    gpio_put(PHASE_C_LOW, 0);

    gpio_pull_down(HALL_A);
    gpio_pull_down(HALL_B);
    gpio_pull_down(HALL_C);

    gpio_set_irq_enabled_with_callback(HALL_A, GPIO_IRQ_EDGE_RISE, true, &hall_a_callback);
    gpio_set_irq_enabled_with_callback(HALL_B, GPIO_IRQ_EDGE_RISE, true, &hall_b_callback);
    gpio_set_irq_enabled_with_callback(HALL_C, GPIO_IRQ_EDGE_RISE, true, &hall_c_callback);


    add_repeating_timer_us(-1000000 / 100,  &timer_callback, NULL, &data_timer);
    while (1) {
        tight_loop_contents();
    }
}
