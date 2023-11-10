#include "sensorAltimeter.hpp"

AltimeterSensor::AltimeterSensor() {


}

bool AltimeterSensor::init() {

    return true;
}


bool AltimeterSensor::getData(void* data) {

    double *altim_data = (double *) data;

    // Get Data (Temp Example)
    *altim_data = 0.36;

    if (1 == 2) {
        Logger::Get().logErr("Altimeter Data Error");
        return false;
    }

    data = (void*) altim_data;
    return true;
}
