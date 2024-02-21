#pragma once
#include <iostream>
#include <exception>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "kalmanfilter.hpp"
#include "logger.hpp"
#include "actuationPlan.hpp"
#include "rocketUtils.hpp"
#include "sensorIMU.hpp"
#include "sensorAltimeter.hpp"
#include "motor.hpp"
#include "eigen3/Eigen/Dense"

using namespace Eigen;

class ADS {

    private:

        KalmanFilter kf;      
        ActuationPlan plan;

        IMUSensor imu;
        AltimeterSensor altimeter;
        Motor motor;
        Vehicle rocket;

        /**
         * @brief Logs a summary of all pertinent current rocket data
         *  (e.g. Altitude, Velocity, Acceleration)
         */
        void logSummary();

        /**
         * @brief Add a csv entry of all pertinent current rocket data
         *  (e.g. Altitude, Velocity, Acceleration)
         * 
         */
        void csvLog();
        
        /**
         * @brief Performs a routine to calculate the average altitude
         *      while the vehicle is waiting on the pad.
         */
        void updateOnPadAltitude();

        /**
         * @brief Update the vehicle with the current sensor (IMU & Altimeter) readings
         */
        void updateSensorData();

        /**
         * @brief Update the rocket state based on its current telemetry readings.
         *     Also Log pertinent telemetry and rocket state data
         * 
         * @return 
         */
        int updateRocketState();

    public:

        /**
         * @brief Construct a new ADS object
         * 
         * @param plan The Actuation Plan for the Rocket
         */
        ADS(ActuationPlan plan);

        /**
         * @brief Run the full active drag system from launch to landing.
         */
        virtual void run();
};