#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "pico/divider.h"

#define SERVO_PIN 0 // Servo motor connected to GPIO PIN 0 (GP0)
#define WRAP_VALUE 65535
#define CLOCK_DIV_RATE 38.15
#define SERVO_MIN 3
#define SERVO_MAX 13
#define SERVO_RANGE (SERVO_MAX - SERVO_MIN)

class PWM {
    public:
        /**
         * @brief Initialize the PWM
         * 
         */
        static void init();
        /**
         * @brief Set the duty cycle of our servo
         * 
         * @param duty_cycle_percent from 0 to 100
         */
        static void set_duty_cycle(int duty_cycle_percent);
        /**
         * @brief Set the servo percent objec to the given percent
         * 
         * @param percent from 0 to 100
         */
        static void set_servo_percent(int percent);
};