#include "pid.hpp"

pid::pid(float Kp, float Ki, float Kd) {
    this->Kp = float_to_s1x14(Kp);
    this->Ki = float_to_s1x14(Ki);
    this->Kd = float_to_s1x14(Kd);
}

s1x14 pid::update(s1x14 setpoint, s1x14 measurement, s1x14 dt) {
    error = (setpoint - measurement);
    if (Kp) {
        proportion = muls1x14(Kp, error); 
    }

    if (Ki) {
        integral = integral + muls1x14(Ki, muls1x14(dt, error));

        if (integral > INT16_MAX) {
            integral = INT16_MAX;
        } else if (integral < INT16_MIN) {
            integral = INT16_MAX;
        }
    }

    if (Kd) {
        int32_t deriv = (error - prev_error) - (int32_t) (setpoint - prev_setpoint);
        prev_setpoint = setpoint;
        prev_error = error;

        if (deriv > INT16_MAX) {
            deriv = INT16_MAX;
        } else if (deriv < INT16_MIN) {
            deriv = INT16_MIN;
        }

        derivative = muls1x14(Kd, deriv);
    }

    int32_t out = ((int32_t) proportion) + ((int32_t) integral) + ((int32_t) derivative);

    if (out > INT16_MAX) {
        out = INT16_MAX;
    } else if (out < INT16_MIN) {
        out = INT16_MIN;
    }
    output = (s1x14) out;
    return output;
};

