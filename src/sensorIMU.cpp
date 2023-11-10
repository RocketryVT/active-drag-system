#include "sensorIMU.hpp"



IMUSensor::IMUSensor() {


}

bool IMUSensor::init() {

    return true;
}


bool IMUSensor::getData(void* data) {

    IMUData *imuData = (IMUData *) data;

    // Get Data (Temp Example) -- For testing
    imuData->acceleration->push_back(3.0);
    imuData->acceleration->push_back(4.0);
    imuData->acceleration->push_back(5.0);

    imuData->linear_acceleration->push_back(20.0);
    imuData->linear_acceleration->push_back(25.0);
    imuData->linear_acceleration->push_back(47.0);

    if (1 == 2) {
        Logger::Get().logErr("IMU Data Error");
        return false;
    }

    data = (void*) imuData;
    return true;
}





