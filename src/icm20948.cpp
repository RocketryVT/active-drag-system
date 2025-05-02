#include "icm20948.hpp"

//Startup routine, configure all 3 sensors and enable them for data output
void ICM20948::initialize() {
    //Dedicatedly disable the chip's sleep mode
    buffer[0] = 0x06;   //PWR_MGMT1
    buffer[1] = 0x00;   //Normal power mode
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    
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
    bypass_mag_i2c();

    //Enable Gyroscope, Accelerometer, and Magnetometer
    
    //Perform sample read??? my mind is mush and full of itussy nonsense
    buffer[0] = R_ICM20948_B0_ACCEL_XOUT_H;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 23, false);
    printf("FIRST THREE BYTES OF MAGNETOMETER (hopefully lol): [%x, %x, %x]\n", buffer[14], buffer[15], buffer[16]);

}

//Configure the auxilary i2c bus for bypass operation, directly connecting it to I2C1 for comms with AK09916 mag sensor
void ICM20948::bypass_mag_i2c() {
    //Force disable the master I2C module
    set_register_bank(0);
    buffer[0] = R_ICM20948_B0_USER_CTRL;
    buffer[1] = 0x00;   //Blow the internal I2C controller's control logic schmoove off
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Configure the internal I2C bus for bypass mode
    buffer[0] = R_ICM20948_B0_INT_PIN_CFG;
    buffer[1] = 0x02;   //BYPASS_EN
    i2c_write_blocking(i2c, addr, buffer, 2, false);

}

//Configure the auxilary i2c bus for proper operation, and then configure the magnetometer *on* that bus
void ICM20948::configure_mag_i2c() {
    //Enable the auxilary I2C bus
    set_register_bank(0);
    buffer[0] = R_ICM20948_B0_USER_CTRL;
    buffer[1] = 0x20;   //TODO: Define this formally in a struct, enables electrical isolation of internal I2C pins??
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Configure the speed and stop conditions of the auxilary I2C bus
    set_register_bank(3);
    buffer[0] = R_ICM20948_B3_I2C_MST_CTRL;
    buffer[1] = 0x07;   //TODO: Define this formally in a struct, configures clock speed and stop conditions
    i2c_write_blocking(i2c, addr, buffer, 2, false);
    sleep_ms(10);

    //Disable I2C bypass???
//    set_register_bank(0);
//    buffer[0] = R_ICM20948_B0_INT_PIN_CFG;
//    buffer[1] = 0x02;   //TODO: Define this formally in a struct, bit is in the interrupt pin config for some reason
//    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Attempt to run a reset on the magnetometer
    printf("ATTEMPTING TO RESET MAGNETOMETER...\n");
    write_aux_register(0x0C, 0x32, 0x01);
    printf("MAGNETOMETER RESET SEQUENCE COMPLETE, MOST LIKELY UNSUCCESSFULLY\n");
    
    //Reset the entire chip??
    buffer[0] = 0x06;   //PWR_MGMT1
    buffer[1] = 0x80;   //DEVICE_RESET bit
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Dedicatedly disable the chip's sleep mode
    buffer[0] = 0x06;   //PWR_MGMT1
    buffer[1] = 0x00;   //Normal power mode
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Every driver I've found does this iterative I2C reset and reenable thing when enabling the magnetometer
    //Seems like a bunk chip design to me but....
    bool aux_i2c_setup_failed = true;
    printf("Beginning setup failure check loop...\n");
    for (int i = 0; i < 10; i++) {
        //Re-enable the auxilary I2C bus, using the same steps as before
        set_register_bank(0);
        buffer[0] = R_ICM20948_B0_USER_CTRL;
        buffer[1] = 0x20;   
        i2c_write_blocking(i2c, addr, buffer, 2, false);
        //Re-configure the auxilary I2C bus, using the same steps as before
        set_register_bank(3);
        buffer[0] = R_ICM20948_B3_I2C_MST_CTRL;
        buffer[1] = 0x07;   
        i2c_write_blocking(i2c, addr, buffer, 2, false);
        sleep_ms(10);
        
        //Attempt to pull chip ID from magnetometer
        uint8_t mag_id_LSB = read_aux_register(0x8C, 0x01);
        uint8_t mag_id_MSB = read_aux_register(0x8C, 0x00);
        uint16_t mag_id = (mag_id_MSB << 8) | mag_id_LSB;
        printf("MAG ID FROM HELPER METHOD: [%x]\n", mag_id);
        
        //Check if the magnetometer ID was read successfully over the auxilary I2C bus
        if (mag_id != ICM20948_MAG_AUX_ID) {
            printf("Resetting master I2C bus!\n");
            
            set_register_bank(0);
            buffer[0] = R_ICM20948_B0_USER_CTRL;
            buffer[1] = 0x02;   //TODO: Define this in the same struct as above, Master I2C module reset bit
            i2c_write_blocking(i2c, addr, buffer, 2, false);
        } else {
            printf("======= Ending setup failed loop, ID check ran successfully! ========\n");
        }
    }

    //TODO: Actually do the magnetometer configuration like for ODR and such
}

//Configure register bank to parameter, used for initialization/configuration of sensors
void ICM20948::set_register_bank(uint8_t bank) {
    buffer[0] = R_ICM20948_REG_BANK_SEL;
    buffer[1] = bank << 4;  //User bank is configured in bits 5:4, 3:0 are reserved
    printf("SWAPPING REGISTER BANK TO (dec, shifted hex): [%i, %x]\n", bank, (bank << 4));
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}

//Write a byte to a slave register on the auxilary (internal) i2c bus of the sensor
void ICM20948::write_aux_register(uint8_t slv_addr, uint8_t slv_reg, uint8_t value) {
    //Swap to register bank 3 so the I2C_SLV4 registers are accessible
    set_register_bank(3);

    //Write the address of the slave device to the SLV4_ADDR register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_ADDR;
    buffer[1] = slv_addr;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Write the value to be written to SLV4_DO register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_DO;
    buffer[1] = value;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Write the register on the slave device that will be written to to the SLV4_REG register
    buffer[0] = R_ICM20948_B3_I2C_SLV4_REG;
    buffer[1] = slv_reg;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Set the enable bit for slave 4 in the SLV4_CTRL register to begin the write operation
    slv4_ctrl.fields.I2C_SLV4_EN = true;
    buffer[0] = R_ICM20948_B3_I2C_SLV4_CTRL;
    buffer[1] = slv4_ctrl.data;
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Wait for the operation to complete by checking the master status bit
    //set_register_bank(0);
    size_t waitCount = 0;
    bool opComplete = false;
    while (!opComplete && waitCount < MAX_SLV4_ACK_CHECKS) {
        //buffer[0] = R_ICM20948_B0_I2C_MST_STATUS;
        buffer[0] = R_ICM20948_B3_I2C_SLV4_CTRL; 
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        slv4_ctrl.data = buffer[0];

        //opComplete = buffer[0] & B_ICM20948_I2C_SLV4_DONE_MASK;
        opComplete = !slv4_ctrl.fields.I2C_SLV4_EN;
        
        printf("Waiting for auxilary I2C write to complete (I2C_SLV4_CTRL): [%x]\n", slv4_ctrl.data);
        waitCount++;
        sleep_ms(1);
    }

    sleep_ms(50);
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
    printf("---- ENABLING SLAVE 4 COMMS: [%x]\n", slv4_ctrl.data);
    i2c_write_blocking(i2c, addr, buffer, 2, false);

    //Wait for the operation to complete by checking the master status bit
    //set_register_bank(0);
    size_t waitCount = 0;
    bool opComplete = false;
    while (!opComplete && waitCount < MAX_SLV4_ACK_CHECKS) {
        //Check the local I2C operation status
        set_register_bank(3);
        buffer[0] = R_ICM20948_B3_I2C_SLV4_CTRL;
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        slv4_ctrl.data = buffer[0];

        //opComplete = (buffer[0] & B_ICM20948_I2C_SLV4_DONE_MASK) >> 4;
        opComplete = !slv4_ctrl.fields.I2C_SLV4_EN;

        //Check the master I2C bus status
        set_register_bank(0);
        buffer[0] = R_ICM20948_B0_I2C_MST_STATUS;
        i2c_write_blocking(i2c, addr, buffer, 1, true);
        i2c_read_blocking(i2c, addr, buffer, 1, false);
        opComplete = buffer[0] >> 6;    //Check SLV4 NACK bit

        printf("Waiting for auxilary I2C read to complete (I2C_SLV4_CTRL, I2C_MST_STATUS) - [%x], [%x]\n", slv4_ctrl.data, buffer[0]);
        waitCount++;
        sleep_ms(1);
    }

    //Once operation is complete, read the output from the... data *input* register (I love TDK I love TDK I love TDK)
    set_register_bank(3);
    //Check register bank??
    buffer[0] = R_ICM20948_REG_BANK_SEL;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    printf("==== CURRENT REGISTER BANK BEFORE READ COMPLETION: [%x]\n", buffer[0]);
    
    buffer[0] = R_ICM20948_B3_I2C_SLV4_DI;
    i2c_write_blocking(i2c, addr, buffer, 1, true);
    i2c_read_blocking(i2c, addr, buffer, 1, false);
    
    return buffer[0];
}

