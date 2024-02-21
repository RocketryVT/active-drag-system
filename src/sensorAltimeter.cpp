#include "../include/sensorAltimeter.hpp"

AltimeterSensor::AltimeterSensor() {

}

//Startup routine copied from Adafruit library, as are most data getting methods
//Adaptation is largely editing for readability and porting from Adafruit_I2C to BBB I2C (sensorI2C.hpp implementation)
bool AltimeterSensor::init(std::string I2C_FILE) {

    // Vehicle *vehicle = (Vehicle *) data;
    // // Do Stuff 
    // data = (void*) vehicle;
    // return true;

    // Check a register with a hard-coded value to see if comms are working
    uint8_t whoami = readSingleRegister(MPL3115A2_WHOAMI);
    if (whoami != MPL3115A2_WHOAMI_EXPECTED) {
        perror("MPL INITIALIZATION DID NOT PASS WHOAMI DEVICE CHECK!")
        return false;
    }

    //Send device dedicated reset byte to CTRL1 Register
    writeSingleRegister(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
    //Wait for reset to wipe its way through device and reset appropriate bit of CTRL1 Register
    while (readSingleRegister(MPL3115A2_CTRL_REG1) & MPL3115A2_CTRL_REG1_RST);

     //Set oversampling (?) and altitude mode by default
    currentMode = MPL3115A2_ALTIMETER;
    ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;
    writeSingleRegister(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);

    //Configure data return types, I don't really understand this chunk but Adafruit does it this way so we will too I guess
    writeSingleRegister(MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
                                    MPL3115A2_PT_DATA_CFG_PDEFE |
                                    MPL3115A2_PT_DATA_CFG_DREM);

    return true;
}

void AltimeterSensor::setMode(mpl3115a2_mode_t mode) {
  ctrl_reg1.reg = readSingleRegister(MPL3115A2_CTRL_REG1);
  ctrl_reg1.bit.ALT = mode;
  writeSingleRegister(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);
  currentMode = mode;
}

// bool AltimeterSensor::getData(void* data) {

//     double *altim_data = (double *) data;

//     // Get Data from somewhere
//     // ....
//     // ....


//     // Example for testing
//     //*altim_data = 0.36;

//     if (1 == 2) {
//         Logger::Get().logErr("Altimeter Data Error");
//         return false;
//     }

//     data = (void*) altim_data; // Is this necessary?
//     return true;
// }
