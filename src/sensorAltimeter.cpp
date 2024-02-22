#include "../include/sensorAltimeter.hpp"

AltimeterSensor::AltimeterSensor(std::string I2C_FILE) {
    this -> I2C_FILE = I2C_FILE;
}

//Startup routine copied from Adafruit library, as is most of the data getting methods
//Adaptation is largely editing for readability and porting from Adafruit_I2C to BBB I2C (sensorI2C.hpp implementation)
bool AltimeterSensor::init() {

    // Vehicle *vehicle = (Vehicle *) data;
    // // Do Stuff 
    // data = (void*) vehicle;

    //Pass file string from parent to setup function, actual I2C bus gets stored internally.
    setupI2C(I2C_FILE);

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

//EXPECTED THAT USER WILL NEVER SET MODE TO PRESSURE AFTER INITIAL CONFIGURATION
void AltimeterSensor::setMode(mpl3115a2_mode_t mode) {
  ctrl_reg1.reg = readSingleRegister(MPL3115A2_CTRL_REG1);
  ctrl_reg1.bit.ALT = mode;
  writeSingleRegister(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);
  currentMode = mode;
}

double AltimeterSensor::getAltitude() {
    //Request new data reading
    requestOneShotReading();
    //If new data is available, read it and store it to internal fields
    if (isNewDataAvailable()) {
        //Logger flag here for new data?
        updateCurrentDataBuffer();
    }
    //Return internal field, whether updated or not
    return internalAltitude;
}

double AltimeterSensor::getTemperature() {
    //Request new data reading
    requestOneShotReading();
    //If new data is available, read it and store it to internal fields
    if (isNewDataAvailable()) {
        //Logger flag here for new data?
        updateCurrentDataBuffer();
    }
    //Return internal field, whether updated or not
    return internalTemperature;
}

void AltimeterSensor::requestOneShotReading() {
    //Request current status of oneshot reading
    ctrl_reg1.reg = readSingleRegister(MPL3115A2_CTRL_REG1);
    //If oneshot is complete, proc a new one; if it isn't, do nothing.
    //THIS PRODUCES DUPLICATE DATA IF READING REQUESTS FROM BB DON'T LINE UP WITH READING COMPLETION ON SENSOR.
    if (!ctrl_reg1.bit.OST) {
        // initiate one-shot measurement
        ctrl_reg1.bit.OST = 1;
        writeSingleRegister(MPL3115A2_CTRL_REG1, ctrl_reg1.reg);
    }
}

void AltimeterSensor::isNewDataAvailable() {
    //Returns PTDR bit of status register, 1 if new data for Temp OR Alt/Pres is available
    //There *are* registers available for exclusively temperature *or* pressure/altitude, but 
    //for simplicity's sake we'll use the combined one for now.
    return ((read8(MPL3115A2_REGISTER_STATUS) & MPL3115A2_REGISTER_STATUS_PTDR) != 0);
}

//Adafruit returns specific field based on input parameter, this method updates all internal fields at once instead
double AltimeterSensor::updateCurrentDataBuffer() {
    uint8_t buffer[5] = {MPL3115A2_REGISTER_PRESSURE_MSB, 0, 0, 0, 0};
    readMultipleRegisters(MPL3115A2_REGISTER_PRESSURE_MSB, 5);

    //Pressure is no longer used, assumed rocket is only logging altitude
    // uint32_t pressure;
    // pressure = uint32_t(buffer[0]) << 16 | uint32_t(buffer[1]) << 8 |
    //         uint32_t(buffer[2]);
    // return double(pressure) / 6400.0;

    //Altitude Conversion
    int32_t alt;
    alt = uint32_t(buffer[0]) << 24 | uint32_t(buffer[1]) << 16 |
        uint32_t(buffer[2]) << 8;
    internalAltitude = double(alt) / 65536.0;

    int16_t t;
    t = uint16_t(buffer[3]) << 8 | uint16_t(buffer[4]);
    internalTemperature = double(t) / 256.0;
}
