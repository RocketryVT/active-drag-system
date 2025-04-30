#include "icm20948.hpp"

void ICM20948::initialize() {
    //Read the value of the WHO_AM_I register and print whether the device was found
    printf("ATTEMPTING COMMUNICATION WITH BREAKOUT SENSOR...\n");
    buffer[0] = R_ICM20948_WHO_AM_I;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    printf("COMMUNICATION COMPLETE!\n");

    if (buffer[0] == B_ICM20948_WHO_AM_I_VALUE) {
        printf("WHO_AM_I value was returned successfully!\n");
    } else {
        printf("WHO_AM_I value was NOT returned successfully, something is wrong!\n");
        printf("Output from read command: [%x]\n\n", buffer[0]);
    }
}
