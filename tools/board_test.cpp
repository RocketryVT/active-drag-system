#include "boards/pico.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"
#include "pico/time.h"

int main() {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    sleep_ms(1500);

    while (1) {
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }
}
