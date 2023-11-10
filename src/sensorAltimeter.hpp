#pragma once
#include "sensor.hpp"
#include "logger.hpp"


class AltimeterSensor : public Sensor {

    private:


    public:

        AltimeterSensor();

        bool init() override;

        bool getData(void* data) override;
};




