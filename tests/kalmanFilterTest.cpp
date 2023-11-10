#include <iostream>
#include <gtest/gtest.h>
#include "eigen3/Eigen/Dense"
#include "../src/kalmanfilter.hpp"

using namespace Eigen;

class KalmanFilterTest : public ::testing::Test {

	protected:

		KalmanFilterTest() {

			kf = new KalmanFilter(2, 1, 1, 1);
		}
			
		//~KalmanFilterTest() {}

		KalmanFilter *kf;
};


/**
 *	@brief Test 
 *
 * **/
TEST_F(KalmanFilterTest, setInitialState) {
	
	VectorXf state_vec(2);
	MatrixXf state_cov(2, 2);
	state_vec << 1, 2;
	state_cov << 1, 3, 4, 9;

	kf->setInitialState(state_vec, state_cov);
	
	
	// ASSERT Statements.....
}

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

	res = kf->run(control, measurement, 1);
	
	
	// ASSERT Statements.....
}




















