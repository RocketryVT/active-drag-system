#include <iostream>
#include <gtest/gtest.h>
#include "eigen3/Eigen/Dense"
#include "../src/kalmanfilter.hpp"

using namespace Eigen;

class KalmanFilterTest : public ::testing::Test {

	protected:

		KalmanFilterTest() {

			kf = new KalmanFilter(2, 1, 1);
		}
			
		//~KalmanFilterTest() {}

		KalmanFilter *kf;
};


/**
 *	@brief Test a single iteration of the Kalman Filter
 *
 * **/
TEST_F(KalmanFilterTest, run) {
	
	VectorXf control(1);
	VectorXf measurement(1);
	control << 1;
	measurement << 1;
	VectorXf res(1);

	res = kf->run(control, measurement);
	
	
	// ASSERT Statements.....
}




















