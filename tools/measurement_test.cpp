
//#include "altimeter.hpp"
#include "high_accel.hpp"
#include "magnetometer.hpp"
#include "mid_imu.hpp"

#define MAX_SCL 100000
#define SDA_GPIO 20
#define SCL_GPIO 21

#define DATA_RATE_HZ 5

//TODO: Migrate new altimeter instantiation and constructors from baseline branch
//Altimeter alt(i2c_default);
HighAccel high(i2c0);
Magnetometer mag(i2c0);
MidIMU imu(i2c0);

void heartbeat(int num_beats, int delay_ms) {
	for (int i = 0; i < num_beats; i++) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
		sleep_ms(delay_ms);
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
		sleep_ms(delay_ms);
	}
}

int main() {
	//Initialize pin handling on RP2040
	stdio_init_all();
	i2c_init(i2c0, MAX_SCL);
	gpio_set_function(SDA_GPIO, GPIO_FUNC_I2C);
	gpio_set_function(SCL_GPIO, GPIO_FUNC_I2C);
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	//Initial heartbeat on boot
	heartbeat(5, 100);
	printf("I2C Initialized successfully!\n");
	getchar();
	printf("Beginning sensor initialization...\n");

	//Initialize all sensors
	//alt.initialize();
	high.initialize();
	printf("High Accel initialized!\n");
	mag.initialize();
	printf("Magnetometer initialized!\n");
	imu.initialize();
	printf("Mid IMU initialized!\n");
	printf("Sensors all initialized successfully!\n");

	//Wait for user input to start main loop
	heartbeat(4, 250);
	printf("Initialization complete! Enter character to begin testing.\n");
	getchar();

	//Main loop
	while (true) {
		//Heartbeat at beginning of cycle
		heartbeat(2, 125);	//500ms
		
		//Request data from sensors
		high.update();
		//TODO: Add update method calls for other sensors after refactor

		//Collect data from sensors
		Vector3f accel_out = high.get_data();
		Vector3f mag_out = mag.getData();
		Vector3f imu_accel_out = imu.get_accel_data();
		Vector3f imu_gyro_out = imu.get_gyro_data();

		printf("ACCELEROMETER: [%4.2f, %4.2f, %4.2f]\n",
			accel_out[0], accel_out[1], accel_out[2]);
		printf("MAGNETOMETER:  [%4.2f, %4.2f, %4.2f]\n",
			mag_out[0], mag_out[1], mag_out[2]);
		printf("IMU (AC, GY):  [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]\n\n",
			imu_accel_out[0], imu_accel_out[1], imu_accel_out[2], imu_gyro_out[0], imu_gyro_out[1], imu_gyro_out[2]);
	}
		
}
