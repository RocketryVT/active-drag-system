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

        void logSummary();
        
        void updateOnPadAltitude();

        void updateSensorData();

        void updateRocketState();

    public:

        ADS(ActuationPlan _plan);

        void run();
};