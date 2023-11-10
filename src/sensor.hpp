#pragma once


class Sensor {
    
    public:

        /**
         * @brief Initialize the sensor. 
         * 
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        virtual bool init() = 0;

        /**
         * @brief Initialize the sensor. 
         * 
         * @param data Data to be obtained from sensor
         * 
         * @return true Sensor read Success
         * @return false Sensor read Failure
         */
        virtual bool getData(void* data) = 0;
};