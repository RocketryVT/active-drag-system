#include "boards/pico_w.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include "pico/divider.h"

#define SERVO_PIN 27 // Servo motor connected to GPIO PIN 32 (GP27)
#define WRAP_VALUE 65535
#define CLOCK_DIV_RATE 38.15
#define SERVO_MIN 13
#define SERVO_MAX 3
#define SERVO_RANGE -10

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
