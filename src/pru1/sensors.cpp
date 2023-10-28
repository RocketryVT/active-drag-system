//
// Created by Gregory Wainer on 10/14/23.
//
#include <string>
#include "barometer.cpp"
#include "imu.cpp"

namespace pru1 {
    class sensors {
    public:
        pru1::barometer bar;
        pru1::imu imu;

        std::string imu_gpio = "";
        std::string baro_gpio = "";
    };
}

int main(int argc, char *argv[]) {

}