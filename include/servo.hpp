#pragma once

#define MICRO_MOTOR_POS_SENS_SDA 14
#define MICRO_MOTOR_POS_SENS_SCL 15
#define MICRO_MOTOR_POS_ADDR 0x06
#define MICRO_MOTOR_POS_REG 0x03

#include "hardware/i2c.h"
#include "mct8316z.hpp"

class servo {
    public:
        servo(i2c_inst_t* inst, mct8316z* motor_driver);

        void initialize();

        void set_position(float angle);

        float get_position();

        void update();

    private:
        uint16_t read_position_sensor() {
            buffer[0] = MICRO_MOTOR_POS_REG;
            i2c_write_blocking(inst, MICRO_MOTOR_POS_ADDR, buffer, 1, true);
            i2c_read_blocking(inst, MICRO_MOTOR_POS_ADDR, buffer, 2, false);
            raw_angle_counts = ((((uint16_t) buffer[0]) << 6) | (((uint16_t) buffer[1]) >> 2));
            return raw_angle_counts;
        };

        i2c_inst_t* inst;

        uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};

        uint16_t raw_angle_counts = 0;
};
