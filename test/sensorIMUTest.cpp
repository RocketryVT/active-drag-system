#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "../include/sensorIMU.hpp"
#include "../include/rocketUtils.hpp"

class IMUSensorTest : public ::testing::Test {

	protected:

		IMUSensorTest() {

			imu = new IMUSensor();
		}
			
		//~IMUSensorTest() {}

		IMUSensor *imu;
};


/**
 *	@brief Test a 
 *
 * **/
TEST_F(IMUSensorTest, init) {

    Vehicle rocket;

    imu->init((void*)&rocket);

	// ASSERT Statements.....
}


/**
 *	@brief Test a 
 *
 * **/
TEST_F(IMUSensorTest, getData) {

    Vehicle rocket;

    imu->getData((void*)&rocket);

	// ASSERT Statements.....
}


// TODO: FIGURE OUT WHY MAKING 'rocket' a POINTER CAUSES A SEGFAULT
