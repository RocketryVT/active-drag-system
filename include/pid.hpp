#pragma once

#include <stdint.h>
#include <stdio.h>

#include "fixed_point.h"

class pid {
    public:
        pid(float Kp, float Ki, float Kd);

        s1x14 update(s1x14 setpoint, s1x14 measurement, s1x14 dt);

    private:
        // Gain Constants
        s1x14 Kp;
        s1x14 Ki;
        s1x14 Kd;

        s1x14 proportion = 0;
        int32_t integral = 0;
        int32_t derivative = 0;

        s1x14 prev_setpoint = 0;
        s1x14 error = 0;
        s1x14 prev_error = 0;
        s1x14 output = 0;
};
