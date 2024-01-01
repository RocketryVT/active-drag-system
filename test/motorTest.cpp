#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "../include/motor.hpp"
#include "../include/rocketUtils.hpp"

class MotorTest : public ::testing::Test {

	protected:

		MotorTest() {

			m1 = new Motor();
		}
			
		//~IMUSensorTest() {}

		Motor *m1;
};


/**
 *	@brief Test a 
 *
 * **/
TEST_F(MotorTest, init) {

    Vehicle rocket;

    m1->init((void*)&rocket);

	// ASSERT Statements.....
}


/**
 *	@brief Test a 
 *
 * **/
TEST_F(MotorTest, writeData) {

    Vehicle rocket;

    m1->writeData((void*)&rocket);

	// ASSERT Statements.....
}


// TODO: FIGURE OUT WHY MAKING 'rocket' a POINTER CAUSES A SEGFAULT
