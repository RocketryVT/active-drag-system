#pragma once
#include <iostream>
#include <vector>
#include <ctime>
#include <vector>
#include <iomanip>
#include <sstream>

// Deployment angle limits
#define MAX_ANGLE 100
#define MIN_ANGLE 120

// Altimeter initialization count limit
#define COUNT_LIMIT 50

// Constants
#define G_0 9.8066

// Threshold limits
#define BOOST_ACCEL_THRESH 3
#define BOOST_HEIGHT_THRESH 20
#define GLIDE_ACCEL_THRESH 0.5

#define ALTI_DEPL_THRESHOLD 0.5
#define RATE_ALTI_DEPL 1/50
#define FSM_DONE_SURFACE_ALTITUDE 200
#define APOGEE_FSM_CHANGE 3

#define INIT_DEPLOYMENT 0 

#define TIME_BO 8
#define TIME_APO 25
#define TIME_END 120

#define PAD_PRESSURE 102250

#define LAUNCH_DATE "4-15-2023"
#define LOG_FILENAME "DataLog_" LAUNCH_DATE ".txt"

#define LED_GAP_TIME 0.5
#define LED_BRIGHTNESS 1

#define TEST_MODE false


enum VehicleState {ON_PAD, BOOST, GLIDE, APOGEE, DONE};
extern std::string state_for_log[5];

struct Vehicle {

	int status;

	std::vector<double> acceleration; // Probably change to vectors
    std::vector<double> linear_acceleration[3];

    double apogee_altitude;
	double previous_altitude;
	double current_altitude;
	double filtered_altitude;

	double filtered_velocity;

	double deployment_angle;

	bool imuInitFail;
	bool imuReadFail;
	bool altiInitFail;
	bool altiReadFail;

	double ON_PAD_altitude;
	bool ON_PAD_fail;

	time_t start_time;
	time_t fail_time; // For failure termination
	time_t liftoff_time;
	time_t relog_time;
	time_t deploy_time; // NOT INITIALIZED YET
	time_t loop_time;
	time_t led_time;
};

double deploy_percentage_to_angle(double percentage);

/**
 * @brief ..........................................................
 * 
 * @param prefix Identifying information ............................
 * @param data Data to Log
 * @param precision The decimal precision value for the data
 * 
 * @return A string with the formatted data.
 */
std::string format_data(std::string prefix, double data, int precision);
