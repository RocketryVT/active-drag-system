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
TEST_F(SurfaceFitModelTest, getFit1) {
	
	double res = surfFM->getFit(1, 2);

	EXPECT_NEAR(res, -771671.3793209707, 0.01);
}

/**
 *	@brief Test a 
 *
 * **/
TEST_F(SurfaceFitModelTest, getFit2) {
	
	double res = surfFM->getFit(33.3, 49);

	EXPECT_NEAR(res, -507325.4658735892, 0.01); 
}