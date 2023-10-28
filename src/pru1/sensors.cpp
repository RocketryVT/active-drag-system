//
// Created by Gregory Wainer on 10/14/23.
//
#include <string>
#include "barometer.cpp"
#include "imu.cpp"

namespace pru1 {
    class sensors {
    public:
        std::string imu_gpio = "";
        int imu_baud_rate = 115200;

        std::string baro_gpio;
        int baro_baud_rate = 115200;


        pru1::imu imu = pru1::imu(imu_gpio, imu_baud_rate);
        pru1::barometer bar = pru1::barometer(baro_gpio, baro_baud_rate);

        static void write_to_sys() {
            printf("%f", pru1::imu_data.x);
            
        }
    };
}

int main(int argc, char *argv[]) {
    pru1::sensors sensors = pru1::sensors();
}