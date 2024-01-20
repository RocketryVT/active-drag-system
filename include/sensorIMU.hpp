#pragma once
#include <vector>
#include "sensor.hpp"
#include "logger.hpp"
#include "rocketUtils.hpp"

struct IMUData {
    std::vector<double> acceleration[3];
    std::vector<double> linear_acceleration[3];
};

class IMUSensor : public Sensor {

    private:


    public:
        /**
         * @brief Construct a new IMUSensor object
         * 
         */
        IMUSensor();

        /**
         * @brief Inititlize the IMU
         * 
         * @param data Data for initializing the sensor
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        bool init(void* data) override;

        /**
         * @brief Read data from the IMU. 
         * 
         * @param data Data to be obtained from IMU
         * @return true Sensor read Success
         * @return false Sensor read Failure
         */
        bool getData(void* data) override;
};