#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "../src/sensorAltimeter.hpp"
#include "../src/rocketUtils.hpp"

class AltimeterSensorTest : public ::testing::Test {

	protected:

		AltimeterSensorTest() {

			alti = new AltimeterSensor();
		}
			
		//~IMUSensorTest() {}

		AltimeterSensor *alti;
};


/**
 *	@brief Test a 
 *
 * **/
TEST_F(AltimeterSensorTest, init) {

    Vehicle rocket;

    alti->init((void*)&rocket);

	// ASSERT Statements.....
}


/**
 *	@brief Test a 
 *
 * **/
TEST_F(AltimeterSensorTest, getData) {

    Vehicle rocket;

    alti->getData((void*)&rocket.current_altitude);

	// ASSERT Statements.....
}


// TODO: FIGURE OUT WHY MAKING 'rocket' a POINTER CAUSES A SEGFAULT
