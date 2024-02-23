#include <iostream>
#include "sensorI2C.hpp"

int main (int argc, char** argv) {
    AltimeterSensor altimeter = new AltimeterSensor("/dev/i2c-2");
    altimeter.init();
    for (int i = 0; i < 1000; i++) {
        std::cout << "Altitude: " << altimeter.getAltitude() << std::endl;
    }
}
