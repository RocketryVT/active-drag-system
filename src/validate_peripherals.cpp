#include "mid_imu.hpp"
#include "high_accel.hpp"
//#include "altimeter.hpp"
#include "magnetometer.hpp"

#define MAX_I2C_CLOCK 400000

#define DATA_RATE_HZ 500
#define LOOP_PERIOD (1.0f / DATA_RATE_HZ)
#define LOG_RATE_HZ 8
#define HEART_RATE_HZ 2

//Pin definitions
#define SDA_GPIO 20
#define SCL_GPIO 21

//Instantiate all peripherals
mid_imu mid(i2c0);
high_accel high(i2c0);
//altimeter alt(i2c0);
magnetometer mag(i2c0);

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

	printf("RP2040 INITIALIZED, BEGINNING PERIPHERAL INITIALIZATION\n");

	//Run all peripheral initialization routines
	mid.initialize();
	printf("IMU INITIALIZED SUCCESSFULLY!\n");
	high.initialize();
	printf("HIGH-G ACCELEROMETER INITIALIZED SUCCESSFULLY!\n");
//	alt.initialize();
//	printf("ALTIMETER INITIALIZED SUCCESSFULLY!\n");
	mag.initialize();
	printf("MAGNETOMETER INITIALIZED SUCCESSFULLY!\n");
	
	printf("PERIPHERAL INITIALIZATION COMPLETE, BEGINNING VALIDATION\n");

	//Run validation for all sensors and confirm
	bool midValid = mid.validate();
	bool highValid = high.validate();
//	bool altValid = alt.validate();
	bool magValid = mag.validate();
	if (!midValid) printf("MID-G IMU VALIDATION FAILED!\n");
	if (!highValid) printf("HIGH-G IMU VALIDATION FAILED!\n");
//	if (!altValid) printf("ALTIMETER VALIDATION FAILED!\n");
	if (!magValid) printf("MAGNETOMETER VALIDATION FAILED!\n");
	printf("COMPLETED PERIPHERAL VALIDATION! IF NO ERROR MESSAGE SENT, PERIPHERAL VALIDATED SUCCESSFULLY\n");

	while (true) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		sleep_ms(125);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		sleep_ms(125);
	}
}
