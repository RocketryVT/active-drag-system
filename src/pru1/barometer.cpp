//
// Created by Gregory Wainer on 10/28/23.
//
#include <boost/asio.hpp>

namespace pru1 {
    class barometer {
    public:
        barometer(const std::string& gpio, int baud_rate){
            readBar(gpio, baud_rate);
        }
    private:

        static void readBar(const std::string& gpio, int baud_rate) {
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

                    // Discard the remaining x bytes
                    serial.read_some(boost::asio::buffer(buffer, 5));
                }
            }
        }

    };
}
