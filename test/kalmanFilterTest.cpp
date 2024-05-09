#include <iostream>
#include <gtest/gtest.h>
#include "eigen3/Eigen/Dense"
#include "../include/kalmanfilter.hpp"

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
 *	@brief Test Setting the initial state x & P
 *
 * **/
TEST_F(KalmanFilterTest, setInitialState) {
	
	VectorXf state_vec(2);
	MatrixXf state_cov(2, 2);
	state_vec << 1, 2;
	state_cov << 1, 3, 4, 9;
	
	// Success Case
	EXPECT_TRUE(kf->setInitialState(state_vec, state_cov));

	// Failure Case
	VectorXf state_vec2(4);
	state_vec2 << 1, 2, 3, 4;
	EXPECT_FALSE(kf->setInitialState(state_vec2, state_cov));
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

	EXPECT_NEAR(0.5454, res(0), 0.0001);
	EXPECT_NEAR(1, res(1), 0.0001);
}


/**
 * @brief Test run() when the time step value is changed between function calls.
 * 
 */
TEST_F(KalmanFilterTest, runChange) {
	
	VectorXf control(1);
	VectorXf measurement(1);
	control << 1;
	measurement << 1;
	VectorXf res(1);

	res = kf->run(control, measurement, 0.1);
	EXPECT_NEAR(0.09545, res(0), 0.00001);
	EXPECT_NEAR(0.1, res(1), 0.1);

	res = kf->run(control, measurement, 0.15);
	EXPECT_NEAR(0.2761, res(0), 0.0001);
	EXPECT_NEAR(0.3585, res(1), 0.0001);
}




















