#pragma once
#include <algorithm>
#include <ctime>
#include "surfaceFitModel.hpp"
#include "rocketUtils.hpp"
#include "sensorIMU.hpp"
#include "sensorAltimeter.hpp"

class ActuationPlan {

    private:
        SurfaceFitModel sFitModel;
        
    public:

        /**
         * @brief Construct a new Actuation Plan object
         * 
         */
        ActuationPlan();

        /**
         * @brief Construct a new Actuation Plan object
         * 
         * @param sFitModel 
         */
        ActuationPlan(SurfaceFitModel sFitModel);

        /**
         * @brief Run the Fin Actuation Plan.
         *  Adjusts the fin angle values depending on the current vehicle state during the launch
         * 
         * @param rocket Provides current rocket status and hold updated fin angle value.
         */
        void runPlan(Vehicle& rocket);
};