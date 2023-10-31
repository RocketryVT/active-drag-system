//
// Created by Gregory Wainer on 10/30/2023.
//

#pragma once

namespace pru1 {
    struct imuData {
        // Angular Velocity
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