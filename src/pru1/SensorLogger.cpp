//
// Created by Gregory Wainer on 10/14/23.
//
#include <iostream>
#include <cstdio>
#include "SensorStructs.h"

namespace pru1 {
    using namespace std;
    class SensorLogger {
    public:
        SensorLogger() {
            freopen( "output.txt", "w", stdout );
            freopen( "error.txt", "w", stderr );
        }

        static void logImu() {
            cout << pru1::imuData.x << pru1::imuData.y << pru1::imuData.z << pru1::imuData.w
                 << pru1::imuData.ax << pru1::imuData.ay << pru1::imuData.az << "\n";
        }

        static void logBaro() {

        }

        static void logAll() {
            logImu();
            logBaro();
        }

        static void closeLogs() {
            fclose(stdout);
            fclose(stderr);
        }
    };
}