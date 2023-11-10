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

        ActuationPlan();
        
        ActuationPlan(SurfaceFitModel _sfitM);

        void runPlan(Vehicle *rocket);
};