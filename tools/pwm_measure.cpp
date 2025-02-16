#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "measure_pwm.pio.h"


#define INPUT_PIN 6
#define OUTPUT_PIN 7 // Encoder Absolute Position GPIO Pin

// function prototypes
void pico_led_init();
void pico_set_led(bool led_on);

/************ MAIN ************/
int main()
{
    // main function local variables
    int32_t input_val = 0;


    // setup function calls
    stdio_init_all();
    pico_led_init(); // Built-in LED setup
    pico_set_led(true);

    gpio_init(OUTPUT_PIN);
    gpio_set_function(OUTPUT_PIN, GPIO_FUNC_PWM);
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
    uint offset = pio_add_program(pio, &measure_pwm_program);
    uint sm = pio_claim_unused_sm(pio, true);
    measure_pwm_program_init(pio, sm, offset, INPUT_PIN);


    /************ FOREVER LOOP ************/
    while (true) {

        // get encoder position from PIO sm
        input_val = measure_pwm_get_timer(pio, sm);
        printf("input_val: %d\n", input_val);

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


