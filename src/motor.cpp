#include "motor.hpp"



Motor::Motor() {


}

bool Motor::init(void* data) {

    Vehicle *vehicle = (Vehicle *) data;
    double duty = 100 - ((MIN_ANGLE / 180) * vehicle->duty_span + DUTY_MIN);

    // Initialize stuff
    // .....
    // .....


    data = (void*) vehicle; // Is this necessary?
    return true;
}


bool Motor::writeData(void* data) {

    Vehicle *vehicle = (Vehicle *) data;
    double duty = 100 - ((vehicle->deployment_angle / 180) * vehicle->duty_span + DUTY_MIN);

    // Send the Data somewhere
    // ..... Pin
    // ..... Duty
    // ..... PWM frequency Hz
    // ..... Polarity
    

    if (1 == 2) {
        Logger::Get().logErr("Some type of Error");
        return false;
    }

    data = (void*) vehicle; // Is this necessary?
    return true;
}


