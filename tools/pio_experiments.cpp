#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/regs/intctrl.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "pio_experiments.pio.h"


#define BLINKY_PIN PICO_DEFAULT_LED_PIN
#define HEART_RATE_HZ 5

void blink_callback();
bool heartbeat_callback(repeating_timer_t *rt);
repeating_timer_t heartbeat_timer;
volatile uint8_t led_counter;

int main()
{
    // setup function calls
    stdio_init_all();

    gpio_init(BLINKY_PIN);
    gpio_set_dir(BLINKY_PIN, GPIO_OUT);
    gpio_put(BLINKY_PIN, 0);

    PIO pio = pio0;
    uint offset = pio_add_program(pio0, &blink_program);
    blink_program_init(pio0, 1, offset, BLINKY_PIN);

    irq_add_shared_handler(PIO0_IRQ_1, blink_callback, 0);
    irq_set_enabled(PIO0_IRQ_1, true);

    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);
    while (true) {
        tight_loop_contents();
    }
}

void blink_callback() {
    printf("Blinked!\n");
    pio_interrupt_clear(pio0, 1);
}

bool heartbeat_callback(repeating_timer_t *rt) {
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    bool led_status = sequence[led_counter];
    gpio_put(BLINKY_PIN, led_status);
    led_counter++;
    led_counter %= sequence_length;
    return true;
}
