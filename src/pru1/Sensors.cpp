//
// Created by Gregory Wainer on 10/14/23.
//
#include <string>
#include "Barometer.cpp"
#include "IMU.cpp"
#include "SensorLogger.cpp"

namespace pru1 {
    struct imuData imuData;
    class Sensors {
    public:
        std::string imu_gpio;
        int imu_baud_rate = 115200;

        std::string baro_gpio;
        int baro_baud_rate = 115200;


        pru1::IMU imu = pru1::IMU(imu_gpio, imu_baud_rate);
        pru1::Barometer bar = pru1::Barometer(baro_gpio, baro_baud_rate);

        static void write_to_sys() {
            printf("%f", pru1::imuData.x);
        }

    };
}
int main(int argc, char *argv[]) {
    pru1::Sensors sensors = pru1::Sensors();
    pru1::SensorLogger::logAll();
    pru1::SensorLogger::closeLogs();
}