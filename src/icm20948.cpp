#include "icm20948.hpp"

//Startup routine, configure all 3 sensors and enable them for data output
void ICM20948::initialize() {
    //Read the value of the WHO_AM_I register and print whether the device was found
    printf("ATTEMPTING COMMUNICATION WITH BREAKOUT SENSOR...\n");
    buffer[0] = R_ICM20948_B0_WHO_AM_I;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    printf("COMMUNICATION COMPLETE!\n");

    if (buffer[0] == B_ICM20948_WHO_AM_I_VALUE) {
        printf("WHO_AM_I value was returned successfully!\n");
    } else {
        printf("WHO_AM_I value was NOT returned successfully, something is wrong!\n");
        printf("Output from read command: [%x]\n\n", buffer[0]);
    }

    //Configure Gyroscope FSR and ODR
    
    //Configure Accelerometer FSR and ODR

    //Configure Magnetometer, just like, generally
    configure_mag_i2c();

    //Enable Gyroscope, Accelerometer, and Magnetometer
    
    //Perform sample read??? my mind is mush and full of itussy nonsense
    buffer[0] = R_ICM20948_B0_ACCEL_XOUT_H;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 23, false);
    printf("FIRST THREE BYTES OF MAGNETOMETER (hopefully lol): [%x, %x, %x]\n", buffer[14], buffer[15], buffer[16]);

    uint8_t mag_id = read_aux_register(0x8C, 0x01);
    printf("MAG ID FROM HELPER METHOD: [%x]\n", mag_id);
}

//Configure the auxilary i2c bus for proper operation, and then configure the magnetometer *on* that bus
void ICM20948::configure_mag_i2c() {
    //Configure the speed and stop conditions of the auxilary I2C bus
    set_register_bank(3);
    buffer[0] = R_ICM20948_B3_I2C_MST_CTRL;
    buffer[1] = 0x17;   //TODO: Define this formally in a struct, configures clock speed and stop conditions
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    
    //Enable the auxilary I2C bus
    set_register_bank(0);
    buffer[0] = R_ICM20948_B0_USER_CTRL;
    buffer[1] = 0x20;   //TODO: Define this formally in a struct, enables electrical isolation of internal I2C pins

    //TODO: Actually do the magnetometer configuration like for ODR and such
}

//Configure register bank to parameter, used for initialization/configuration of sensors
void ICM20948::set_register_bank(uint8_t bank) {
    buffer[0] = R_ICM20948_REG_BANK_SEL;
    buffer[1] = bank;
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

//Write a byte to a slave register on the auxilary (internal) i2c bus of the sensor
void ICM20948::write_aux_register(uint8_t slv_addr, uint8_t slv_reg, uint8_t value) {
    //Swap to register bank 3 so the I2C_SLV4 registers are accessible
    set_register_bank(3);

    //Write the value to be written to SLV4_DO register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_DO;
    buffer[1] = value;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Write the address of the slave device to the SLV4_ADDR register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_ADDR;
    buffer[1] = slv_addr;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Write the register on the slave device that will be written to to the SLV4_REG register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_REG;
    buffer[1] = slv_reg;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Set the enable bit for slave 4 in the SLV4_CTRL register
    slv4_ctrl.fields.I2C_SLV4_EN = true;
    buffer[0] = R_ICM20948_B3_I2C_SLV4_CTRL;
    buffer[1] = slv4_ctrl.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Wait for the operation to complete by checking the master status bit
    set_register_bank(0);
    size_t waitCount = 0;
    bool opComplete = false;
    while (!opComplete) {
        buffer[0] = R_ICM20948_B0_I2C_MST_STATUS;
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        opComplete = buffer[0] & B_ICM20948_I2C_SLV4_DONE_MASK;

        printf("Waiting for auxilary I2C write to complete...\n");
        sleep_ms(1);
    }
}

//Read a byte from a slave register on the auxilary (internal) i2c bus of the sensor
uint8_t ICM20948::read_aux_register(uint8_t slv_addr, uint8_t slv_reg) {
    //Swap to register bank 3 so the I2C_SLV4 registers are accessible
    set_register_bank(3);

    //Add read bit to the slave address
    slv_addr |= 0x80;

    //Write the address of the slave device to the SLV4_ADDR register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_ADDR;
    buffer[1] = slv_addr;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Write the register on the slave device that will be written to to the SLV4_REG register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_REG;
    buffer[1] = slv_reg;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Set the enable bit for slave 4 in the SLV4_CTRL register
    slv4_ctrl.fields.I2C_SLV4_EN = true;
    buffer[0] = R_ICM20948_B3_I2C_SLV4_CTRL;
    buffer[1] = slv4_ctrl.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Wait for the operation to complete by checking the master status bit
    set_register_bank(0);
    size_t waitCount = 0;
    bool opComplete = false;
    while (!opComplete) {
        buffer[0] = R_ICM20948_B0_I2C_MST_STATUS;
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        opComplete = buffer[0] & B_ICM20948_I2C_SLV4_DONE_MASK;

        printf("Waiting for auxilary I2C read to complete - [%x]\n", buffer[0]);
        sleep_ms(1);
    }

    //Once operation is complete, read the output from the... data *input* register (I love TDK I love TDK I love TDK)
    set_register_bank(3);
    buffer[0] = R_ICM20948_B3_I2C_SLV4_DI;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    
    set_register_bank(0);    
    return buffer[0];
}

