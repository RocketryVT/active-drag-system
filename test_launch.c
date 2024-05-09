// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// MPL3115A2
// This code is designed to work with the MPL3115A2_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/products

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <time.h>

#include <stdint.h>
#include <sys/mman.h>

#define BILLION 1000000000L;

#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU0_DRAM		0x00000			// Offset to DRAM
#define PRU1_DRAM		0x02000
#define PRU_SHAREDMEM	0x10000			// Offset to shared memory

#define CYCLES_PER_SECOND 200000000
#define CYCLES_PER_PERIOD (CYCLES_PER_SECOND / 800)
#define PRU_TIMER_PASSCODE 0x31138423

volatile uint32_t	*pru0DRAM_32int_ptr;		// Points to the start of local DRAM
volatile uint32_t	*pru1DRAM_32int_ptr;		// Points to the start of local DRAM
volatile uint32_t	*prusharedMem_32int_ptr;	// Points to the start of the shared memory

typedef enum {
    PAD,
    MOTOR_BURN,
    COAST,
    APOGEE,
    RECOVERY,
    END
} rocket_state;

typedef enum {
    RETRACT,
    EXTEND
} ads_state;

int start_pwm_count(uint32_t duty_cycle) {
	// write to PRU shared memory
	pru0DRAM_32int_ptr[0] = duty_cycle;	// On time
	return 0;
}

void update_state(rocket_state* state, int base_altitude, int current_alt, int prev_alt, double time_delta) {
    double velocity = ((current_alt - prev_alt) / time_delta);
    static double motor_burn_time = 0.0f;
    static double recovery_time = 0.0f;
    switch (*state) {
        case PAD:
            if (velocity >= 50.0f) {
                *state = MOTOR_BURN;
                pru0DRAM_32int_ptr[1] = PRU_TIMER_PASSCODE;
            }
            break;
        case MOTOR_BURN:
            if (motor_burn_time >= 1.8f) {
                *state = COAST;
                break;
            }
            motor_burn_time += time_delta;
            break;
        case COAST:
            if (velocity <= -50.0f) {
                *state = APOGEE;
            }
            break;
        case APOGEE:
            *state = RECOVERY;
            break;
        case RECOVERY:
            if (recovery_time >= 30.0f) {
                *state = END;
                break;
            }
            recovery_time += time_delta;
            break;
    }
}

void update_ads(ads_state state) {
    switch (state) {
        case RETRACT:
            start_pwm_count(15);
            break;
        case EXTEND:
            start_pwm_count(3);
            break;
    }
}

void main()  {
    int base_altitude = -1;
    int curr_altitude = 0;
    int prev_altitude = 0;
    rocket_state r_state = PAD;
    struct timespec start, current, previous;
    double time_delta;

    system("echo 'stop' > /sys/class/remoteproc/remoteproc1/state");
    system("cp -r /home/debian/bbb_pru_test_act/pru_pwm /lib/firmware/am335x-pru0-fw");
    system("echo 'start' > /sys/class/remoteproc/remoteproc1/state");

	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-2";
	if((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, TSL2561 I2C address is 0x60(96)
	ioctl(file, I2C_SLAVE, 0x60);

	// Select control register(0x26)
	// Active mode, OSR = 128, altimeter mode(0xB9)
	char config[2] = {0};
	config[0] = 0x26;
	config[1] = 0xB9;
	write(file, config, 2);
	// Select data configuration register(0x13)
	// Data ready event enabled for altitude, pressure, temperature(0x07)
	config[0] = 0x13;
	config[1] = 0x07;
	write(file, config, 2);
	// Select control register(0x26)
	// Active mode, OSR = 128, altimeter mode(0xB9)
	config[0] = 0x26;
	config[1] = 0xB9;
	write(file, config, 2);
	sleep(1);

////////////////////////// SERVO SHIT
	uint32_t *pru;		// Points to start of PRU memory.
	int	fd;
	printf("Servo tester\n");
	
	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		printf ("ERROR: could not open /dev/mem.\n\n");
	}
	pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED) {
		printf ("ERROR: could not map memory.\n\n");
	}
	close(fd);
	pru0DRAM_32int_ptr =     pru + PRU0_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU0 memory
	pru1DRAM_32int_ptr =     pru + PRU1_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU1 memory
	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;	// Points to start of shared memory

////////////////////////////////
    while (base_altitude == -1) {
        char reg[1] = {0x00};
        write(file, reg, 1);
        char data[4] = {0};
        if(read(file, data, 4) == 4) {
            base_altitude = (((data[1] << 16) | (data[2] << 8) | (data[3] & 0xF0)) >> 8);
            curr_altitude = base_altitude;
            prev_altitude = base_altitude;
            break;
        }
    }
    clock_gettime( CLOCK_REALTIME, &start);
    clock_gettime( CLOCK_REALTIME, &current);
    FILE* log = fopen("ads_log", "a");
    int pad_logged = 0;
    while (1) {
        // Read 6 bytes of data from address 0x00(00)
        // status, tHeight msb1, tHeight msb, tHeight lsb, temp msb, temp lsb
        char reg[1] = {0x00};
        write(file, reg, 1);
        char data[4] = {0};
        if(read(file, data, 4) != 4) {
            printf("Error : Input/Output error \n");
            continue;
        }

        // Convert the data
        prev_altitude = curr_altitude;
        curr_altitude = (((data[1] << 16) | (data[2] << 8) | (data[3] & 0xF0)) >> 8);
        previous = current;
        clock_gettime( CLOCK_REALTIME, &current);
        time_delta = ( current.tv_sec - previous.tv_sec ) + ((double) ( current.tv_nsec - previous.tv_nsec )) / BILLION;

        update_state(&r_state, base_altitude, curr_altitude, prev_altitude, time_delta);
        double time_since_start = ( current.tv_sec - start.tv_sec ) + ((double) ( current.tv_nsec - start.tv_nsec )) / BILLION;

        switch (r_state) {
            case PAD:
                update_ads(RETRACT);
		if (pad_logged) {
		    fprintf(log, "%.2f - PAD: ", time_since_start);
		    pad_logged = 1;
		}
                break;
            case MOTOR_BURN:
                update_ads(RETRACT);
                fprintf(log, "%.2f - MOTOR_BURN: ", time_since_start);
                break;
            case COAST:
                update_ads(EXTEND);
                fprintf(log, "%.2f - COAST: ", time_since_start);
                break;
            case APOGEE:
                update_ads(RETRACT);
                fprintf(log, "%.2f - APOGEE: ", time_since_start);
                break;
            case RECOVERY:
                update_ads(RETRACT);
                fprintf(log, "%.2f - RECOVERY: ", time_since_start);
                break;
            case END:
                break;
        }

    	fprintf(log, "Altitude: %d m  Velocity: %.2f  Time Delta: %.2f\n", curr_altitude, ((curr_altitude - prev_altitude) / time_delta), time_delta);
        fflush(log);
        usleep(250000);
    }

    close(file);
    fclose(log);
    if(munmap(pru, PRU_LEN)) {
		printf("munmap failed\n");
	} else {
		printf("munmap succeeded\n");
	}
	exit(0);
}

