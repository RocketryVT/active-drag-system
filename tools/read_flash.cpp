#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/gpio.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"

/* Priorities at which the tasks are created. */
#define	HEARTBEAT_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
// #define	SERIAL_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )

static void heartbeat_task( void *pvParameters );
// static void serial_task( void *pvParameters );

void vApplicationTickHook(void) { /* optional */ }
void vApplicationMallocFailedHook(void) { /* optional */ }
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) { for( ;; ); }

int main() {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    xTaskCreate(heartbeat_task, "heartbeat", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
}

static void heartbeat_task( void *pvParameters ) {
    static volatile uint8_t led_counter = 0;
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;
    while (1) {
        bool led_status = sequence[led_counter];
        gpio_put(PICO_DEFAULT_LED_PIN, led_status);
        led_counter++;
        led_counter %= sequence_length;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
