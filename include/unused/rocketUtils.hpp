#pragma once
#include <iostream>
#include <vector>
#include <ctime>
#include <vector>
#include <iomanip>
#include <sstream>
#include <fstream>

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

#define DUTY_MAX 14.5
#define DUTY_MIN 3

#define LAUNCH_DATE "4-15-2023"
#define LOG_FILENAME "DataLog_" LAUNCH_DATE ".txt"

#define LED_GAP_TIME 0.5
#define LED_ONE_PATH "/sys/class/leds/beaglebone:green:usr1"
#define LED_BRIGHTNESS_FILE "brightness"
#define LED_FILENAME LED_ONE_PATH LED_BRIGHTNESS_FILE

#define TEST_MODE false


enum VehicleState {ON_PAD, BOOST, GLIDE, APOGEE, DONE};
extern std::string state_for_log[5];

struct Vehicle {

	int status;

	std::vector<double> acceleration; 
    std::vector<double> linear_acceleration;

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

	double duty_span;

	double dt;

	int led_brightness;

	time_t start_time;
	time_t fail_time; // For failure termination
	time_t liftoff_time;
	time_t relog_time;
	time_t deploy_time; // NOT INITIALIZED YET
	time_t loop_time;
	time_t led_time;
};

/**
 * @brief Convert fin deployment percentage to fin rotation angle
 * 
 * @param percentage Fin deployment percentage
 * @return double 
 */
double deploy_percentage_to_angle(double percentage);

/**
 * @brief Set the decimal precision of the given data, and return it 
 * 		as a formatted string with a prefix containing a relevant description of the data. 
 * 
 * @param prefix Identifying or clarifying information about the loggef data
 * @param data Data to Log
 * @param precision The decimal precision value for the data
 * 
 * @return A string with the formatted data.
 */
std::string format_data(std::string prefix, double data, int precision);

/**
 * @brief Blink Beaglebone LED 1
 * 
 * @param vehicle Holds settings pertinent to the Beaglebone LED
 * @return true Successful Blink
 * @return false Unsuccessful Blink
 */
bool led_out(Vehicle *vehicle);
