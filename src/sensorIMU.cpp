#include "sensorIMU.hpp"



IMUSensor::IMUSensor() {


}

bool IMUSensor::init(void* data) {

    return true;
}


bool IMUSensor::getData(void* data) {

    Vehicle *vehicle = (Vehicle *) data;

    // Get Data from somewhere
    // ....
    // ....


    // Example for testing
    // vehicle->acceleration->push_back(3.0);
    // vehicle->acceleration->push_back(4.0);
    // vehicle->acceleration->push_back(5.0);

    // vehicle->linear_acceleration->push_back(20.0);
    // vehicle->linear_acceleration->push_back(25.0);
    // vehicle->linear_acceleration->push_back(47.0);

    if (1 == 2) { // Temporary bullshit
        Logger::Get().logErr("IMU Data Error");
        return false;
    }

    data = (void*) vehicle; // Is this necessary?
    return true;
}





