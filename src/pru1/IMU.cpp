//
// Created by Gregory Wainer on 10/28/23.
//
#include <boost/asio.hpp>
#include <iostream>
#include <boost/thread.hpp>

#include "AsyncSerial.h"
#include "SensorStructs.h"

using namespace std;
using namespace boost;

namespace pru1 {
    /*
    * This is for PRU GPIO access of the Adafruit BNO055 IMU
    */
    class IMU {
    public:
        IMU(const std::string& gpio, int baud_rate){
            readIMU(gpio, baud_rate);
        }
    private:
        static void readIMU(const std::string& gpio, int baud_rate) {
            asio::io_service io;
            asio::serial_port serial(io, gpio);

            serial.set_option(asio::serial_port_base::baud_rate());

            // Send Data
            serial.write_some(asio::buffer("", 2));
            serial.write_some(asio::buffer("", 6));

            while(serial.is_open()) {
                // Read Data
                char buffer[9];
                size_t bytesRead = serial.read_some(asio::buffer(buffer));
                if (true) {
                    // Add Logic to map buffer to object
                    pru1::imuData.x = buffer[0];
                    pru1::imuData.y = buffer[1];
                    pru1::imuData.z = buffer[2];
                    pru1::imuData.w = buffer[3];

                    pru1::imuData.ax = buffer[4];
                    pru1::imuData.ay = buffer[5];
                    pru1::imuData.az = buffer[6];


                    // Discard the remaining x bytes
                    serial.read_some(asio::buffer(buffer, 5));
                    //serial.async_read_some(asio::buffer(buffer, 5), serial.native_handle());
                }
            }


//            try {
//                BufferedAsyncSerial bufferedAsyncSerial("/dev/ttyUSB0", 115200);
//
//                //Return immediately. String is written *after* the function returns,
//                //in a separate thread.
//                bufferedAsyncSerial.writeString("Hello world\n");
//
//                //Simulate doing something else while the serial device replies.
//                //When the serial device replies, the second thread stores the received
//                //data in a buffer.
//                boost::this_thread::sleep(posix_time::seconds(2));
//
//                //Always returns immediately. If the terminator \r\n has not yet
//                //arrived, returns an empty string.
//                cout << bufferedAsyncSerial.readStringUntil("\r\n") << endl;
//
//                bufferedAsyncSerial.close();
//
//            } catch(boost::system::system_error& e)
//            {
//                cout<<"Error: "<<e.what()<<endl;
//                return;
//            }
        }

    };
}