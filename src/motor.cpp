#include "motor.hpp"



Motor::Motor() {


}

bool Motor::init() {

    return true;
}


bool Motor::writeData(void* data) {


    // Send the Data somewhere
    // .....
    // .....


    if (1 == 2) {
        Logger::Get().logErr("Some type of Error");
        return false;
    }

    return true;
}


