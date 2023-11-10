#include <iostream>
#include <gtest/gtest.h>
#include "../src/surfaceFitModel.hpp"
#include "../src/actuationPlan.hpp"


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
 *	@brief Test a 
 *
 * **/
TEST_F(ActuationPlanTest, runPlan) {
	
	Vehicle rocket;

	rocket.status = ON_PAD;

    rocket.apogee_altitude = 0;
    rocket.previous_altitude = 0;
    rocket.current_altitude = 0;
    rocket.filtered_altitude = 0;

    rocket.filtered_velocity = 0;

    rocket.imuInitFail = false;
    rocket.imuReadFail = false;
    rocket.altiInitFail = false;
    rocket.altiReadFail = false;

	plan->runPlan(&rocket);


	
	// ASSERT Statements.....
}