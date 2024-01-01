#include <iostream>
#include <gtest/gtest.h>
#include "eigen3/Eigen/Dense"
#include "../include/surfaceFitModel.hpp"

using namespace Eigen;

class SurfaceFitModelTest : public ::testing::Test {

	protected:

		SurfaceFitModelTest() {

			surfFM = new SurfaceFitModel();
		}
			
		//~SurfaceFitModelTest() {}

		SurfaceFitModel *surfFM;
};


/**
 *	@brief Test a 
 *
 * **/
TEST_F(SurfaceFitModelTest, getFit) {
	
	double res = surfFM->getFit(2, 3);

	EXPECT_NEAR(res, 1, 0.01); // Finish this
}