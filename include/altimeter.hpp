#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <cstdint>

class altimeter {
    private:
        uint8_t altitude_buffer[4];
        uint8_t buffer[4];
        uint8_t addr;
        i2c_inst_t* inst;
    public:
        altimeter(i2c_inst_t* inst, uint8_t addr);

        void initialize();

        void initialize(float threshold_altitude, uint8_t interrupt_pin, gpio_irq_callback_t callback);

        void set_threshold_altitude(float threshold_altitude, uint8_t interrupt_pin, gpio_irq_callback_t callback);

        void unset_threshold_altitude(uint8_t interrupt_pin);

        float get_altitude_converted();

        void get_altitude_raw(uint8_t* buffer);

        uint32_t expose_buffer(uint8_t** buffer);
};

