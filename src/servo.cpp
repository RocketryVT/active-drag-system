#include "servo.hpp"

servo::servo(i2c_inst_t* inst, mct8316z* motor_driver) {
    this->inst = inst;
    this->motor_driver = motor_driver;
}
