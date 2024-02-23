#include "../include/sensorIMU.hpp"

IMUSensor::IMUSensor(std::string I2C_FILE) {
    this -> I2C_FILE = I2C_FILE;
}

bool IMUSensor::init(void* data) {

    //I2C_File passed on object creation, stored in sensorI2C parent
    setupI2C(I2C_FILE);
    
    //In the adafruit code there's a big step of waiting for timeout and connection stuff for up to a full second
    //I don't do that here because the BBB takes like 17 years to boot so we'll just hope it goes faster than that

    //Sanity check for factory device ID 
    uint8_t id = read8(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
        perror("DEVICE ID DID NOT PASS SANITY CHECK FOR BNO IMU!");
        return false;
    }

    //Set default operating mode of IMU into config from startup (will be set properly after config phase)
    setMode(OPERATION_MODE_CONFIG);

    //Writes 1 to the system reset bit in the trigger register
    writeSingleRegister(BNO055_SYS_TRIGGER_ADDR, 0x20);
    //Wait for reset to complete by doing sanity check again
    while (readSingleRegister(BNO055_CHIP_ID_ADDR) != BNO055_ID);

    //Set power mode for sensor
    writeSingleRegister(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);

    //Sensor chip uses two "pages" to multiplex register values
    //Page 0 contains the sensor data (not configuration), which is what we want
    writeSingleRegister(BNO055_PAGE_ID_ADDR, 0);

    //Genuinely no idea why Adafruit does this, ensuring all triggers are off before mode config I guess
    writeSingleRegister(BNO055_SYS_TRIGGER_ADDR, 0x0);

    setMode(default_mode);
    
    return true;
}


bool IMUSensor::getData(void* data) {

    Vehicle *vehicle = (Vehicle *) data;

    // Get Data from somewhere
    // ....
    // ....


    // Example for testing
    // vehicle->acceleration->push_back(3.0);
    // vehicle->acceleration->push_back(4.0);
    // vehicle->acceleration->push_back(5.0);

    // vehicle->linear_acceleration->push_back(20.0);
    // vehicle->linear_acceleration->push_back(25.0);
    // vehicle->linear_acceleration->push_back(47.0);

    if (1 == 2) { // Temporary bullshit
        Logger::Get().logErr("IMU Data Error");
        return false;
    }

    data = (void*) vehicle; // Is this necessary?
    return true;
}





