#pragma once
#include "actuator.hpp"
#include "logger.hpp"
#include "rocketUtils.hpp"

class Motor : public Actuator {

    private:


    public:

        /**
         * @brief Construct a new Motor object
         * 
         */
        Motor();

        /**
         * @brief Initialize the motor. 
         * 
         * @param data Data for initializing the motor
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        virtual bool init(void* data) override;

        /**
         * @brief Write data to the motor.
         * 
         * @param data Data to be writen to the motor
         * @return true Motor write Success
         * @return false Motor write Failure
         */
        virtual bool writeData(void* data) override;
};