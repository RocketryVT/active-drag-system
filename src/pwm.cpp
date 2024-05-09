#include "pwm.hpp"

void PWM::init() {
    // Tell GPIO 0 they are allocated to the PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);

    // Configure PWM slice and set it running
    pwm_config cfg = pwm_get_default_config();
    // Set the PWM clock divider to 38.15
    pwm_config_set_clkdiv(&cfg, CLOCK_DIV_RATE);
    // Set the PWM wrap value to 65535
    pwm_set_wrap(slice_num, WRAP_VALUE);
    // Set the PWM duty cycle to 0 and enable the PWM
    pwm_init(slice_num, &cfg, true);

    // Enable the PWM again cause idk
    pwm_set_enabled(slice_num, true);
}

void PWM::set_duty_cycle(int duty_cycle_percent) {
    // Calculate the raw value
    uint32_t raw_value = WRAP_VALUE * (duty_cycle_percent / 100.0);

    // Set the duty cycle
    pwm_set_gpio_level(SERVO_PIN, raw_value);
}

void PWM::set_servo_percent(int percent) {
    // Calculate the value by clamping the percent from 0 to 100
    // to the SERVO_MIN and SERVO_MAX
    uint32_t value = ((percent * SERVO_RANGE) / 100) + SERVO_MIN;
    PWM::set_duty_cycle(value);
}
