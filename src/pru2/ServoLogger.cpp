//
// Created by Gregory Wainer on 10/30/2023.
//
#include <iostream>
#include <cstdio>
#include "ServoStructs.h"

namespace pru2 {
    using namespace std;
    class ServoLogger {
    public:
        ServoLogger() {
            freopen( "output.txt", "w", stdout );
            freopen( "error.txt", "w", stderr );
        }

        static void logServo() {
            cout << pru2::servoData.angle << pru2::servoData.pwm << "\n";
        }

        static void closeLogs() {
            fclose(stdout);
            fclose(stderr);
        }
    };
}