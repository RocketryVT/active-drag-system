#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "measure_pwm.pio.h"


#define OUTPUT_PIN 7

// function prototypes
void pico_led_init();
void pico_set_led(bool led_on);
void callback();

const uint32_t timing_interval_ms =  (1000 / TIMING_PULSE_FREQUENCY * TIMING_PULSE_RATIO);

/************ MAIN ************/
int main()
{
    // setup function calls
    stdio_init_all();
    pico_led_init(); // Built-in LED setup
    pico_set_led(true);

    gpio_init(INPUT_PULSE_PIN);
    gpio_set_dir(INPUT_PULSE_PIN, GPIO_IN);
    gpio_pull_down(INPUT_PULSE_PIN);
    gpio_init(SIDESET_PIN);

    gpio_init(OUTPUT_PIN);
    gpio_set_function(OUTPUT_PIN, GPIO_FUNC_PWM);
    gpio_init(TIMING_PULSE_PIN);
    gpio_set_function(TIMING_PULSE_PIN, GPIO_FUNC_PWM);
    uint32_t freq = 5000;
    uint32_t duty_c = 32768;
    uint slice_num = pwm_gpio_to_slice_num(OUTPUT_PIN);
    uint16_t wrap_val = UINT16_MAX;
    pwm_config config = pwm_get_default_config();
    float clock_get_hz_var = ((float) clock_get_hz(clk_sys));
    printf("clock_get_hz: %4.2f\n", clock_get_hz_var);
    float div = clock_get_hz_var / (((float) freq) * UINT16_MAX);
    for (; (div < 1.0f); ) {
        duty_c /= ((div + 1) / div);
        wrap_val /= ((div + 1) / div);
        div += 1;
    }
    printf("div: %4.2f\n", div);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, wrap_val); 
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(OUTPUT_PIN, duty_c); //connect the pin to the pwm engine and set the on/off level. 

    // encoder PIO setup
    PIO pio = pio0;
    uint offset = pio_add_program(pio0, &pulse_counter_pio_program);
    pulse_counter_pio_program_init(pio0, 0, offset, INPUT_PULSE_PIN);

    offset = pio_add_program(pio, &timing_pulse_pio_program);
    timing_pulse_pio_program_init(pio, 1, offset, TIMING_PULSE_PIN);
    pio_set_irq1_source_enabled(pio, pis_interrupt1, true);
    irq_add_shared_handler(PIO0_IRQ_1, callback, 0);
    irq_set_enabled(PIO0_IRQ_1, true);


    /************ FOREVER LOOP ************/
    while (true) {

        if (!pio_sm_is_rx_fifo_empty(pio, 1)) {
            if (pio_sm_get_blocking(pio, 1) == 1) {
                if (!pio_sm_is_rx_fifo_empty(pio, 0)) {
                    uint32_t pulse_count = pio_sm_get_blocking(pio, 0);
                    pulse_count = (0x100000000 - pulse_count) & 0xFFFFFFFF;
                    printf("Freq: %d\n", (pulse_count / timing_interval_ms * 1000));
                }
            }
        }
        sleep_ms(1000);


    }
    /************ END FOREVER LOOP ************/
    
}
/************ END MAIN ************/




/************ FUNCTION DEFINITIONS ************/
// initialize the built-in Pico LED
void pico_led_init() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

// turns the built-in Pico LED on/off
void pico_set_led(bool led_on) {
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
}

void callback() {
    uint32_t duty_c = 32768;
    uint slice_num = pwm_gpio_to_slice_num(TIMING_PULSE_PIN);
    pwm_set_enabled(slice_num, false);
    uint16_t wrap_val = UINT16_MAX;
    pwm_config config = pwm_get_default_config();
    float div = 238.422f;
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, wrap_val); 
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(TIMING_PULSE_PIN, duty_c); //connect the pin to the pwm engine and set the on/off level. 
    pwm_set_enabled(slice_num, true);
}
