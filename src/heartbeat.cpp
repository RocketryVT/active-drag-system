#include "heartbeat.hpp"

static int led_pin;

#if (USE_FREERTOS != 1)
static repeating_timer_t heartbeat_timer;

static bool heartbeat_callback(repeating_timer_t *rt) {
    static volatile uint8_t led_counter = 0;
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;
    bool led_status = sequence[led_counter];
    gpio_put(led_pin, led_status);
    led_counter++;
    led_counter %= sequence_length;
    return true;
}
#endif

void heartbeat_initialize(int gpio) {
    led_pin = gpio;
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_put(led_pin, 0);
#if (USE_FREERTOS != 1)
    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);
#endif
}


#if (USE_FREERTOS == 1)
void heartbeat_task( void *pvParameters ) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / HEART_RATE_HZ);
    static volatile uint8_t led_counter = 0;
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        bool led_status = sequence[led_counter];
        gpio_put(PICO_DEFAULT_LED_PIN, led_status);
        led_counter++;
        led_counter %= sequence_length;
    }
}
#endif
