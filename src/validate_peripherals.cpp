#include "mid_imu.hpp"
#include "high_accel.hpp"
//#include "altimeter.hpp"
#include "magnetometer.hpp"

#define MAX_I2C_CLOCK 100000

#define DATA_RATE_HZ 500
#define LOOP_PERIOD (1.0f / DATA_RATE_HZ)
#define LOG_RATE_HZ 8
#define HEART_RATE_HZ 2

//Pin definitions
#define SDA_GPIO 20
#define SCL_GPIO 21

//Instantiate all peripherals
MidIMU mid(i2c0);
HighAccel high(i2c0);
//Altimeter alt(i2c0);
Magnetometer mag(i2c0);

//HELPER FUNCTION FROM PICO SDK
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

//Run all the initialization routines for each of the peripherals and confirm that they're all running properly
int main() {
	//Initialize all necessary RP2040 functions
	stdio_init_all();
	i2c_init(i2c0, MAX_I2C_CLOCK);

	//Configure GPIO as necessary
	gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
	gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);

	//Initial slow heartbeat
	gpio_init(PICO_DEFAULT_LED_PIN);	//Board LED connected to pin 25, same as pico default
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	sleep_ms(500);
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
	sleep_ms(500);
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	
	//Wait for some input
	getchar();
	
	//Confirmation faster heartbeat
	sleep_ms(250);
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
	sleep_ms(250);
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	sleep_ms(250);
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
	sleep_ms(250);
	gpio_put(PICO_DEFAULT_LED_PIN, 1);
	sleep_ms(1000);
	
	printf("\n-------- I2C BUS SCAN, EXAMPLE FROM PICO SDK --------\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("-------- I2C BUS SCAN COMPLETE! --------\n\n");
	printf("RP2040 INITIALIZED, BEGINNING PERIPHERAL INITIALIZATION\n");

	//Run all peripheral initialization routines
	high.initialize();
	printf("HIGH-G ACCELEROMETER INITIALIZED SUCCESSFULLY!\n");
	mag.initialize();
	printf("MAGNETOMETER INITIALIZED SUCCESSFULLY!\n");
	mid.initialize();
	printf("IMU INITIALIZED SUCCESSFULLY!\n");
	//	alt.initialize();
//	printf("ALTIMETER INITIALIZED SUCCESSFULLY!\n");
	
	printf("PERIPHERAL INITIALIZATION COMPLETE, BEGINNING VALIDATION\n");

	//Run validation for all sensors and confirm
	bool highValid = high.validate();
	if (!highValid) printf("HIGH-G ACCEL VALIDATION FAILED!\n");
	bool magValid = mag.validate();
	if (!magValid) printf("MAGNETOMETER VALIDATION FAILED!\n");
	bool midValid = mid.validate();
	if (!midValid) printf("MID-G IMU VALIDATION FAILED!\n");
	//	bool altValid = alt.validate();
//	if (!altValid) printf("ALTIMETER VALIDATION FAILED!\n");
	printf("COMPLETED PERIPHERAL VALIDATION! IF NO ERROR MESSAGE SENT, PERIPHERAL VALIDATED SUCCESSFULLY\n");

	while (true) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		sleep_ms(125);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		sleep_ms(125);
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		sleep_ms(125);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		sleep_ms(125);

		Vector3f accel_out = high.getData();
		Vector3f mag_out = mag.getData();
		Vector6f imu_out = mid.getData();

		printf("ACCELEROMETER: [%4.2f, %4.2f, %4.2f]\n",
				accel_out[0], accel_out[1], accel_out[2]);
		printf("MAGNETOMETER:  [%4.2f, %4.2f, %4.2f]\n",
				mag_out[0], mag_out[1], mag_out[2]);
		printf("IMU (AC, GY):  [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]\n\n",
				imu_out[0], imu_out[1], imu_out[2], imu_out[3], imu_out[4], imu_out[5]);
	}
}
