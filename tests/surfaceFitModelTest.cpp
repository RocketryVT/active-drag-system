#include <iostream>
#include <gtest/gtest.h>
#include "eigen3/Eigen/Dense"
#include "../src/surfaceFitModel.hpp"

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
TEST_F(SurfaceFitModelTest, run) {
	
	
	// ASSERT Statements.....
}