#pragma once
#include "sensorI2C.hpp"
#include <string>

/*
Header file for driver for MPL3115a2 breakout, based heavily on https://github.com/adafruit/Adafruit_MPL3115A2_Library/tree/master
Based on register declarations found https://cdn-shop.adafruit.com/datasheets/1893_datasheet.pdf
Designed using subclass for I2C handler for Beaglebone Black
*/

class AltimeterSensor : public sensorI2C {


    private:
        double internalTemperature = 0;
        double internalAltitude = 0;
        
        //ENUMS COPIED DIRECTLY FROM ADAFRUIT IMPLEMENTATION
        /** MPL3115A2 registers **/
        enum {
            MPL3115A2_REGISTER_STATUS = (0x00),

            MPL3115A2_REGISTER_PRESSURE_MSB = (0x01),
            MPL3115A2_REGISTER_PRESSURE_CSB = (0x02),
            MPL3115A2_REGISTER_PRESSURE_LSB = (0x03),

            MPL3115A2_REGISTER_TEMP_MSB = (0x04),
            MPL3115A2_REGISTER_TEMP_LSB = (0x05),

            MPL3115A2_REGISTER_DR_STATUS = (0x06),

            MPL3115A2_OUT_P_DELTA_MSB = (0x07),
            MPL3115A2_OUT_P_DELTA_CSB = (0x08),
            MPL3115A2_OUT_P_DELTA_LSB = (0x09),

            MPL3115A2_OUT_T_DELTA_MSB = (0x0A),
            MPL3115A2_OUT_T_DELTA_LSB = (0x0B),

            MPL3115A2_WHOAMI = (0x0C),
            //This is hard-coded in the device from the factory
            MPL3115A2_WHOAMI_EXPECTED = (0xC4),

            MPL3115A2_BAR_IN_MSB = (0x14),
            MPL3115A2_BAR_IN_LSB = (0x15),

            MPL3115A2_OFF_H = (0x2D),
        };

        /** MPL3115A2 status register BITS **/
        enum {
            MPL3115A2_REGISTER_STATUS_TDR = 0x02,
            MPL3115A2_REGISTER_STATUS_PDR = 0x04,
            MPL3115A2_REGISTER_STATUS_PTDR = 0x08,
        };

        /** MPL3115A2 PT DATA register BITS **/
        enum {
            MPL3115A2_PT_DATA_CFG = 0x13,
            MPL3115A2_PT_DATA_CFG_TDEFE = 0x01,
            MPL3115A2_PT_DATA_CFG_PDEFE = 0x02,
            MPL3115A2_PT_DATA_CFG_DREM = 0x04,
        };

        /** MPL3115A2 control registers **/
        enum {
            MPL3115A2_CTRL_REG1 = (0x26),
            MPL3115A2_CTRL_REG2 = (0x27),
            MPL3115A2_CTRL_REG3 = (0x28),
            MPL3115A2_CTRL_REG4 = (0x29),
            MPL3115A2_CTRL_REG5 = (0x2A),
        };

        /** MPL3115A2 control register BITS **/
        enum {
            MPL3115A2_CTRL_REG1_SBYB = 0x01,
            MPL3115A2_CTRL_REG1_OST = 0x02,
            MPL3115A2_CTRL_REG1_RST = 0x04,
            MPL3115A2_CTRL_REG1_RAW = 0x40,
            MPL3115A2_CTRL_REG1_ALT = 0x80,
            MPL3115A2_CTRL_REG1_BAR = 0x00,
        };

        /** MPL3115A2 oversample values **/
        enum {
            MPL3115A2_CTRL_REG1_OS1 = 0x00,
            MPL3115A2_CTRL_REG1_OS2 = 0x08,
            MPL3115A2_CTRL_REG1_OS4 = 0x10,
            MPL3115A2_CTRL_REG1_OS8 = 0x18,
            MPL3115A2_CTRL_REG1_OS16 = 0x20,
            MPL3115A2_CTRL_REG1_OS32 = 0x28,
            MPL3115A2_CTRL_REG1_OS64 = 0x30,
            MPL3115A2_CTRL_REG1_OS128 = 0x38,
        };

        /** MPL3115A2 measurement modes **/
        typedef enum {
            MPL3115A2_BAROMETER = 0,
            MPL3115A2_ALTIMETER,
        } mpl3115a2_mode_t;

        /** MPL3115A2 measurement types **/
        typedef enum {
            MPL3115A2_PRESSURE,
            MPL3115A2_ALTITUDE,
            MPL3115A2_TEMPERATURE,
        } mpl3115a2_meas_t;

        //This never actually gets used, and I can't find anything in the datasheet about it??
        #define MPL3115A2_REGISTER_STARTCONVERSION (0x12) ///< start conversion

        //Store current operating mode, sent to device during startup procedure
        //This is why an enum is used rather than raw #define statements
        mpl3115a2_mode_t currentMode;

        //Struct for storing ctrl register contents, copied from adafruit implementation
        typedef union {
            struct {
                uint8_t SBYB : 1;
                uint8_t OST : 1;
                uint8_t RST : 1;
                uint8_t OS : 3;
                uint8_t RAW : 1;
                uint8_t ALT : 1;
            } bit;
            uint8_t reg;
        } CTRL_REG_1_STRUCT;
        //Create instance of this register config to use during device startup and operation
        CTRL_REG_1_STRUCT ctrl_reg1;

    public:

        /**
         * @brief Construct a new Altimeter Sensor object
         * 
         */
        AltimeterSensor(std::string I2C_FILE);

        /**
         * @brief Initialize the Altimeter
         * 
         * @param data Data for initializing the sensor
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        bool init() override;

        //Data getters and setters
        // double getPressure();
        double getAltitude();
        double getTemperature();
        double setSeaLevelPressure(double pressure);

        //Data and mode handlers
        //Use altimeter mode by default as this is what rocket logger wants
        void setMode(mpl3115a2_mode_t mode = MPL3115A2_ALTIMETER);
        void requestOneShotReading();
        bool isNewDataAvailable();
        void updateCurrentDataBuffer();


};




