#pragma once

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <string>

class sensorI2C {
    

    //Predominantly used for I2C handler functions; implement high-level functions in sensor classes
    //Implemented as a combination of Jazz' implementation and Derek Malloy's code
    public:
        
        //Initial single byte write, used at beginning of read operation
        //Returns 0 if successful, -1 if not
        int initialWrite(unsigned char registerAddress) {
            unsigned char *convertedAddressBuffer = new unsigned char[1];
            convertedAddressBuffer[0] = registerAddress;
            //Expect 1 byte response from 1 byte write
            if (write(i2c_bus, convertedAddressBuffer, 1) != 1) {
                fprintf(stderr, "ERROR DOING INITIAL READ TRANSACTION WRITE TO REGISTER %x FOR DEVICE %x\n", registerAddress, deviceAddress);
                return 0;
            }
            return 1;
        }

        int writeRegister(unsigned char registerAddress, unsigned char value) {
            //initialWrite() not used here because it's easier to just pack it into one buffer for file writing
            unsigned char writeBuffer[2];
            writeBuffer[0] = registerAddress;
            writeBuffer[1] = value;

            //Expect 2 byte output
            if (write(i2c_bus, writeBuffer, 2) != 2) {
                //These error messages are kind of obtuse but I'd rather have too much information than not enough
                fprintf(stderr, "ERROR WRITING %x TO REGISTER %x ON DEVICE %x\n", value, registerAddress, deviceAddress);
                return -1;
            }
            return 0;
        }

        //Could probably be uint8_t but Derek Malloy does it with unsigned chars and that's what worked during testing so I don't want to touch it
        unsigned char readSingleRegister(unsigned char registerAddress) {
            printf("reg addr: %X\n", registerAddress);

            initialWrite(registerAddress);
            unsigned char* readBuffer = new unsigned char[1];
            if (read(i2c_bus, readBuffer, 1) != 1){
                fprintf(stderr, "FAILED TO READ VALUE FROM REGISTER %x ON DEVICE %x\n", registerAddress, deviceAddress);
                return -1;
            }
            printf("readBuffer: %X\n", readBuffer[0]);
            return readBuffer[0];
        }

        unsigned char* readMultipleRegisters(unsigned char startingRegisterAddress, int numberOfRegisters) {            
            initialWrite(startingRegisterAddress);
            unsigned char* readBuffer = new unsigned char[numberOfRegisters];
            if (read(i2c_bus, readBuffer, numberOfRegisters) != numberOfRegisters) {
                fprintf(stderr, "ERROR TRYING TO READ %d REGISTERS STARTING AT ADDRESS %x ON DEVICE %x\n", 
                    numberOfRegisters, startingRegisterAddress, deviceAddress);
            }
            return readBuffer;
        }


        //Intakes device address and file 
        //Private because IT'S ASSUMED PROGRAMMER WILL CALL THIS METHOD DURING INIT() ROUTINE
        int setupI2C(std::string I2C_FILE) {
            // Open i2c driver file
            i2c_bus = open("/dev/i2c-2", O_RDWR);
            if(i2c_bus < 0){
                fprintf(stderr, "FAILED TO OPEN I2C BUS USING FILE %s\n", "/dev/i2c-2");
                close(i2c_bus);
                return -1;
            }

            // Identify slave device address (MODIFIED FROM INITIAL IMPLEMENTATION, USES INTERNAL deviceAddress INSTEAD OF PARAMETER)
            if(ioctl(i2c_bus, I2C_SLAVE, deviceAddress) < 0){
                fprintf(stderr, "FAILED TO CONNECT TO DEVICE AT ADDRESS %x VIA I2C\n", deviceAddress);
                close(i2c_bus);
                return -1;
            }
            return 0;
        }

        /**
         * @brief Initialize the sensor. 
         * 
         * @param data Data for initializing the sensor
         * 
         * @return true Initialization Success
         * @return false Initialization Failure
         */
        virtual bool init() = 0;

        unsigned char deviceAddress;
        int i2c_bus;
        std::string I2C_FILE;
};
