//
// Created by Gregory Wainer on 10/28/23.
//
#include <boost/asio.hpp>

struct imu_data {
    // Angular Velocity
    double x;
    double y;
    double z;
    double w;

    // Linear Acceleration
    double ax;
    double ay;
    double az;
};

namespace pru1 {
    static imu_data imu_data;
    class imu {
    public:
        imu(const std::string& gpio, int baud_rate){
            readIMU(gpio, baud_rate);
        }
    private:
        static void readIMU(const std::string& gpio, int baud_rate) {
            boost::asio::io_service io;
            boost::asio::serial_port serial(io, gpio);

            serial.set_option(boost::asio::serial_port_base::baud_rate());

            // Send Data
            serial.write_some(boost::asio::buffer("", 2));
            serial.write_some(boost::asio::buffer("", 6));

            while(serial.is_open()) {
                // Read Data
                char buffer[9];
                size_t bytesRead = serial.read_some(boost::asio::buffer(buffer));
                if (true) {
                    // Add Logic to map buffer to object
                    imu_data.x = buffer[0];
                    imu_data.y = buffer[1];
                    imu_data.z = buffer[2];
                    imu_data.w = buffer[3];

                    imu_data.ax = buffer[4];
                    imu_data.ay = buffer[5];
                    imu_data.az = buffer[6];


                    // Discard the remaining x bytes
                    serial.read_some(boost::asio::buffer(buffer, 5));
                }
            }
        }

    };
}
