#include "../src/sensorIMU.hpp"
#include "../src/sensorAltimeter.hpp"
#include <iostream>
#include <vector>




int main(int argc, char* argv[]) {

    IMUSensor imu = IMUSensor();
    IMUData data;

    imu.getData((void*)&data);

    std::cout << data.acceleration << std::endl;
    std::cout << data.linear_acceleration << std::endl;

    AltimeterSensor alti = AltimeterSensor();

    double res;
    alti.getData((void*)&res);

    std::cout << res << std::endl;

    // TODO: FIGURE OUT WHY MAKING 'data' AND 'res' POINTERS CAUSES A SEGFAULT
}