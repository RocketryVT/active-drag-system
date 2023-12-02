#include "actuationPlan.hpp"

ActuationPlan::ActuationPlan() {}

ActuationPlan::ActuationPlan(SurfaceFitModel sFitModel) : sFitModel(sFitModel) {

}


void ActuationPlan::runPlan(Vehicle& rocket) {


    if (rocket.imuReadFail || rocket.altiReadFail) {
        rocket.deployment_angle = deploy_percentage_to_angle(0); // No fin deployment
    }

    rocket.fail_time = time(nullptr);

    // 2024 Mission---------------------------------------------------------------------
    if (rocket.status == GLIDE) {

        // Fin deployment based on current drag coefficient value
        try {
            double cd = sFitModel.getFit(rocket.filtered_velocity, rocket.filtered_altitude);
            cd = std::min(std::max(0.0, cd), 100.0);
            rocket.deployment_angle = deploy_percentage_to_angle(cd);
        }

        // Full deployment during coasting
        catch (...) {
            rocket.deployment_angle = deploy_percentage_to_angle(0);
            
            if ((time(nullptr) - rocket.deploy_time) > 2 && (time(nullptr) - rocket.deploy_time) < 7) {
                rocket.deployment_angle = deploy_percentage_to_angle(100);
            }
        }        
    }

    else if (rocket.status == APOGEE) {

        rocket.deployment_angle = deploy_percentage_to_angle(50);
    }

    else {

        rocket.deploy_time = time(nullptr);
    }
    // End 2024 Mission------------------------------------------------------------------
}











