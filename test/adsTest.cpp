#include <iostream>
#include <gtest/gtest.h>
#include "../include/surfaceFitModel.hpp"
#include "../include/actuationPlan.hpp"
#include "../include/ads.hpp"

class ADSTest : public ::testing::Test {

	protected:

		ADSTest() {

            SurfaceFitModel sfm = SurfaceFitModel();
            ActuationPlan plan = ActuationPlan(sfm);
			ads = new ADS(plan);
		}
			
		//~ADSTest() {}

		ADS *ads;
};


/**
 *	@brief Test a 
 *
 * **/
TEST_F(ADSTest, run) {
	
	ads->run();

	
	// ASSERT Statements.....
}