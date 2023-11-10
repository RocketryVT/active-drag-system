#pragma once
#include "sensor.hpp"
#include "logger.hpp"
#include "rocketUtils.hpp"


class AltimeterSensor : public Sensor {

    private:


    public:

        AltimeterSensor();

        bool init(void* data) override;

        bool getData(void* data) override;
};




