#pragma once
#include <vector>
#include "sensor.hpp"
#include "logger.hpp"

struct IMUData {
    std::vector<double> acceleration[3];
    std::vector<double> linear_acceleration[3];
};

class IMUSensor : public Sensor {

    private:


    public:

        IMUSensor();

        bool init() override;

        bool getData(void* data) override;
};