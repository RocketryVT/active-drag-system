#include <iostream>
#include <gtest/gtest.h>
#include "../include/surfaceFitModel.hpp"
#include "../include/actuationPlan.hpp"
#include "../include/rocketUtils.hpp"


class ActuationPlanTest : public ::testing::Test {

	protected:

		ActuationPlanTest() {

            SurfaceFitModel sfm = SurfaceFitModel();
			plan = new ActuationPlan(sfm);
		}
			
		//~ActuationPlanTest() {}

		ActuationPlan *plan;
};


/**
 *	@brief Tests running the Actuation Plan
 *
 * **/
TEST_F(ActuationPlanTest, runPlan) {
	
	Vehicle rocket;

    rocket.imuInitFail = false;
    rocket.imuReadFail = false;
    rocket.altiInitFail = false;
    rocket.altiReadFail = false;

    // Test when Vehicle Status: Glide
    rocket.fail_time = (time_t)(-1);
    rocket.deploy_time = time(nullptr);
    rocket.status = GLIDE;
    rocket.filtered_altitude = 1;
    rocket.filtered_velocity = 2;
	plan->runPlan(rocket);
    EXPECT_NEAR(rocket.deployment_angle, 120.0, 0.01);
    EXPECT_NE(rocket.fail_time, (time_t)(-1));

    // Test when Vehicle Status: Apogee
    rocket.deploy_time = time(nullptr);
    rocket.status = APOGEE;
    rocket.filtered_altitude = 1;
    rocket.filtered_velocity = 2;
	plan->runPlan(rocket);
    EXPECT_NEAR(rocket.deployment_angle, 110.0, 0.01);
}