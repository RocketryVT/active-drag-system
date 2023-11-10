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
	
	// ASSERT Statements.....
}