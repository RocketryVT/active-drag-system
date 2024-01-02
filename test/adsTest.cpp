#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../include/surfaceFitModel.hpp"
#include "../include/actuationPlan.hpp"
#include "../include/ads.hpp"


class MockADS : public ADS {

	public:
		
		// ADSTest() {

        //     SurfaceFitModel sfm = SurfaceFitModel();
        //     ActuationPlan plan = ActuationPlan(sfm);
		// 	ads = new ADS(plan);
		// }
			
		// //~ADSTest() {}

		// ADS *ads;

		MockADS(ActuationPlan plan) : ADS(plan) {}

		// Private----------------------
		MOCK_METHOD(void, logSummary, (), (override));
		MOCK_METHOD(void, updateOnPadAltitude, (), (override));
		MOCK_METHOD(void, updateSensorData, (), (override));
		MOCK_METHOD(void, updateRocketState, (), (override));

		// Public----------------------
		MOCK_METHOD(void, run, (), (override));

		// Real Non-Mocked Version
		void RealRun() { ADS::run(); }
};


/**
 *	@brief Test a 
 *
 * **/
TEST(ADSTest, run) {
	
	SurfaceFitModel sfm = SurfaceFitModel();
	ActuationPlan plan = ActuationPlan(sfm);
	MockADS ads(plan);
	
	ads.run();

	
	// ASSERT Statements.....
}