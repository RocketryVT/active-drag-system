#pragma once
#include "sensor.hpp"
#include "logger.hpp"
#include "rocketUtils.hpp"


class AltimeterSensor : public Sensor {

    private:


    public:

        /**
         * @brief Construct a new Altimeter Sensor object
         * 
         */
        AltimeterSensor();

        /**
         * @brief Initialize the Altimeter
         * 
         * @param data Data for initializing the sensor
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        bool init(void* data) override;

        /**
         * @brief Read data from the Altimeter. 
         * 
         * @param data Data to be obtained from Altimeter
         * @return true Sensor read Success
         * @return false Sensor read Failure
         */
        bool getData(void* data) override;
};




