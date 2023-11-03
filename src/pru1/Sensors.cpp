//
// Created by Gregory Wainer on 10/14/23.
//
#include <string>
#include "Barometer.cpp"
#include "IMU.cpp"

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

        void uart_init()
        {
            CT_UART.DLL = 104; /* divisor latch low */
            CT_UART.DLH = 0;   /* divisor latch high - aka DLM*/
            CT_UART.MDR_bit.OSM_SEL = 0; /* 16x oversampling */
            CT_UART.LCR_bit.WLS = 3; /* word length select; 0b11 = 8 bits */

            CT_UART.FCR_bit.FIFOEN = 1; /* FIFO enable */
            CT_UART.FCR_bit.RXCLR = 1; /* receiver FIFO reset */
            CT_UART.FCR_bit.TXCLR = 1; /* transmitter FIFO reset */
            CT_UART.PWREMU_MGMT_bit.URRST = 1; /* enable transmitter */
            CT_UART.PWREMU_MGMT_bit.UTRST = 1; /* enable receiver */
        }

    };
}
int main(int argc, char *argv[]) {
    pru1::Sensors sensors = pru1::Sensors();
}