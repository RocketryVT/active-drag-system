//
// Created by Gregory Wainer on 10/30/2023.
//

#pragma once

namespace pru1 {
    struct imuData {
        // Absolute Orientation Euler Vector
        double aoEX;
        double aoEY;
        double aoEZ;

        // Absolute Orientation Quaternion
        double aoQX;
        double aoQY;
        double aoQZ;
        double aoQW;

        // Angular Velocity (rad/s)
        double x;
        double y;
        double z;
        double w;

        // Linear Acceleration
        double ax;
        double ay;
        double az;
    };
    extern imuData imuData;
}