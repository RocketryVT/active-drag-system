#pragma once


class Actuator {

    private:


    public:

        /**
         * @brief Initialize the actuator. 
         * 
         * @param data Data for initializing the actuator
         * 
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        virtual bool init(void* data) = 0;

        /**
         * @brief Pass data to the actuator. 
         * 
         * @param data Data to sent to the actuator
         * 
         * @return true Actuator write Success
         * @return false Actuator write Failure
         */
        virtual bool writeData(void* data) = 0;
};