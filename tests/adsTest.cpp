#include <iostream>
#include <gtest/gtest.h>
#include "../src/surfaceFitModel.hpp"
#include "../src/actuationPlan.hpp"
#include "../src/ads.hpp"

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
		
	// ASSERT Statements.....
}