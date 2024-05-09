#include "../include/ads.hpp"


// Private----------------------------------------------------------------------
void ADS::logSummary() {
    
    std::string output_string = "" + state_for_log[rocket.status];

    if (!rocket.altiInitFail && !rocket.altiReadFail) {

        output_string += format_data(" ", rocket.filtered_altitude, 3);     
    }

    output_string += format_data(" ", rocket.deployment_angle, 2);    

    if (!rocket.imuInitFail && !rocket.imuReadFail) {

        output_string += format_data(" ", rocket.acceleration[2], 2);    
        output_string += format_data(" ", rocket.filtered_velocity, 2);    
    }

    Logger::Get().log(output_string);
}


void ADS::updateOnPadAltitude() {

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    double avg_alt = 0;
    double alt_read_count = 0;

    while (alt_read_count < COUNT_LIMIT) {
        
        altimeter.getData(&rocket.current_altitude);
        alt_read_count++;
        avg_alt = (avg_alt * (alt_read_count - 1) + rocket.current_altitude) / alt_read_count;
    }
 
    Logger::Get().log(format_data("pad altitude initialization complete - ", avg_alt, 3));
    rocket.ON_PAD_altitude = avg_alt;
}


void ADS::updateSensorData() {

    if (!rocket.imuInitFail) {
        
        try {
            imu.getData((void*)&rocket);
        }

        catch (...) {
            std::exception_ptr e = std::current_exception();
            Logger::Get().logErr(e.__cxa_exception_type()->name());
            rocket.imuReadFail = true;
        }
    }  

    rocket.previous_altitude = rocket.current_altitude; // Why was this placed here????
    
    if (!rocket.altiInitFail) {
        
        try {
            altimeter.getData((void*)&rocket.current_altitude);
            if (rocket.ON_PAD_fail) {
                rocket.ON_PAD_altitude = rocket.current_altitude;
                rocket.ON_PAD_fail = false;
            }

            rocket.altiReadFail = false;
        }

        catch (...) {
            std::exception_ptr e = std::current_exception();
            Logger::Get().logErr(e.__cxa_exception_type()->name());
            rocket.altiReadFail = true;
        }
    }
}


void ADS::updateRocketState() {

    // Filter sensor data
    VectorXf control_input(1);
    VectorXf measurement(1);
    control_input << rocket.acceleration[2];
    measurement << rocket.current_altitude;
    VectorXf filtered = kf.run(control_input, measurement, rocket.dt);
    rocket.filtered_altitude = filtered(0);
    rocket.filtered_velocity = filtered(1);

    if (rocket.apogee_altitude < rocket.filtered_altitude) {
        rocket.apogee_altitude = rocket.filtered_altitude;
    }

    // (VEHICLE ON PAD)
    if (rocket.status == ON_PAD) {

        // If launch detected
        if (rocket.acceleration[2] >= BOOST_ACCEL_THRESH * G_0 
            && rocket.filtered_altitude >= BOOST_HEIGHT_THRESH + rocket.ON_PAD_altitude) {
            Logger::Get().log(format_data("LOM at -- ", (double)(rocket.liftoff_time - rocket.start_time), 3));
        }

        if (TEST_MODE && time(nullptr) - rocket.start_time >= 15) {
            Logger::Get().log(format_data("TEST LOM at -- ", (double)(rocket.liftoff_time - rocket.start_time), 3));
        }

        if (time(nullptr) - rocket.relog_time > 2*60*60 
            && rocket.status == ON_PAD) {
            std::cout << "OverWR Success" << std::endl;
        }
    }

    // (VEHICLE BOOSTING)
    else if (rocket.status == BOOST) {

        if (rocket.acceleration[2] <= GLIDE_ACCEL_THRESH * G_0 
            || time(nullptr) - rocket.liftoff_time >= TIME_BO) {
            rocket.status = GLIDE;
        }
        
    }

    // (VEHICLE IN GLIDE)
    else if (rocket.status == GLIDE) {

        if (rocket.filtered_altitude < rocket.apogee_altitude - APOGEE_FSM_CHANGE 
            || time(nullptr) - rocket.liftoff_time >= TIME_BO + TIME_APO) {
            rocket.status = APOGEE;
            Logger::Get().log(format_data("APO: ", (double)(rocket.apogee_altitude), 2));
        }
    }

    // (VEHICLE AT APOGEE)
    else if (rocket.status == APOGEE) {

        if (rocket.filtered_altitude <= FSM_DONE_SURFACE_ALTITUDE + rocket.ON_PAD_altitude) {
            rocket.status = DONE;
            return;
        }
    }
}


// Public----------------------------------------------------------------------
ADS::ADS(ActuationPlan plan) : plan(plan) {

    rocket.status = ON_PAD;

    rocket.apogee_altitude = 0;
    rocket.previous_altitude = 0;
    rocket.current_altitude = 0;
    rocket.filtered_altitude = 0;

    rocket.filtered_velocity = 0;

    rocket.duty_span = DUTY_MAX - DUTY_MIN;
    rocket.deployment_angle = deploy_percentage_to_angle(INIT_DEPLOYMENT); 

    rocket.dt = 0.1;

    rocket.imuInitFail = false;
    rocket.imuReadFail = false;
    rocket.altiInitFail = false;
    rocket.altiReadFail = false;

    rocket.ON_PAD_altitude = 0;
    rocket.ON_PAD_fail = false;

    rocket.start_time = time(nullptr);
    rocket.fail_time = rocket.start_time;
    rocket.relog_time = rocket.start_time;
    rocket.led_time = rocket.start_time;

    imu = IMUSensor();
    altimeter = AltimeterSensor();
    motor = Motor();
    kf = KalmanFilter(2, 1, 1, rocket.dt);

    Logger::Get().openLog(LOG_FILENAME);
    
    motor.init(&rocket);

    imu.init(nullptr);
    altimeter.init(nullptr);

    if (TEST_MODE) {

        Logger::Get().log("TEST Record Start --");
    }
}



void ADS::run() {

    if (!rocket.altiInitFail) {
        try {
            updateOnPadAltitude();
        }

        catch (...) {
            std::exception_ptr e = std::current_exception();
            Logger::Get().logErr(e.__cxa_exception_type()->name());
            rocket.ON_PAD_fail = true;
        }
    }

    rocket.loop_time = time(nullptr);
    while (rocket.status != DONE) {

        updateSensorData();

        if (!rocket.imuInitFail && !rocket.altiInitFail) {

            updateRocketState();

            // Run the Actuation Plan----------------------------------
            plan.runPlan(rocket);

            if (rocket.imuReadFail || rocket.altiReadFail) {

                if (rocket.imuReadFail) {
                    imu.init(nullptr); // Restart
                    Logger::Get().log("Altimeter reset attempt");
                }

                if (rocket.altiReadFail) {
                    altimeter.init(nullptr); // Restart
                    Logger::Get().log("IMU reset attempt");
                }
            }
        }

        // Altimeter or IMU setup failed. Attempt to reinitialize
        else {

            if (time(nullptr) - rocket.fail_time >= TIME_END) {
                rocket.status = DONE;
            }

            if (rocket.altiInitFail || rocket.altiReadFail) {
                imu.init(nullptr); // Restart
                Logger::Get().log("Altimeter reset attempt");
            }

            if (rocket.imuInitFail || rocket.imuReadFail) {
                altimeter.init(nullptr); // Restart
                Logger::Get().log("IMU reset attempt");
            }

            rocket.deployment_angle = deploy_percentage_to_angle(INIT_DEPLOYMENT);
        }

        // Actuate Servos
        motor.writeData(&rocket);

        logSummary();

        // Blink Beaglebone LED 1
        if (time(nullptr) - rocket.led_time > LED_GAP_TIME) {
            led_out(&rocket);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        rocket.dt = time(nullptr) - rocket.loop_time;
        rocket.loop_time = time(nullptr);
    }

    Logger::Get().closeLog();
    std::cout << "Done" << std::endl;
}











