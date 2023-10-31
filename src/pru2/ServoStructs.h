//
// Created by gregw on 10/30/2023.
//

#pragma once

namespace pru2 {
    struct servoData {
        double angle;
        int pwm;
    };
    extern servoData servoData;
}