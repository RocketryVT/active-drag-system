#pragma once


class Sensor {
    
    public:

        /**
         * @brief Initialize the sensor. 
         * 
         * @param data Data for initializing the sensor
         * 
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        virtual bool init(void* data) = 0;

        /**
         * @brief Read data from the sensor. 
         * 
         * @param data Data to be obtained from sensor
         * 
         * @return true Sensor read Success
         * @return false Sensor read Failure
         */
        virtual bool getData(void* data) = 0;
};