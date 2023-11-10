#include "sensor.hpp"

bool Sensor::init() {

}

bool Sensor::altimeterInit(float pad_pressure) {

    // Altimeter Setup
    altimeter.setMode(MPL3115A2_ALTIMETER);


    altimeter.setSeaPressure(pad_pressure);

}