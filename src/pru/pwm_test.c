/* 
 *
 *  pwm tester
 *	The on cycle and off cycles are stored in each PRU's Data memory
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>

#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU0_DRAM		0x00000			// Offset to DRAM
#define PRU1_DRAM		0x02000
#define PRU_SHAREDMEM	0x10000			// Offset to shared memory

#define CYCLES_PER_SECOND 200000000
#define CYCLES_PER_PERIOD (CYCLES_PER_SECOND / 800)
#define PRU_TIMER_PASSCODE 0x31138423

uint32_t	*pru0DRAM_32int_ptr;		// Points to the start of local DRAM
uint32_t	*pru1DRAM_32int_ptr;		// Points to the start of local DRAM
uint32_t	*prusharedMem_32int_ptr;	// Points to the start of the shared memory

/*******************************************************************************
* int start_pwm_count(int ch, int countOn, int countOff)
* 
* Starts a pwm pulse on for countOn and off for countOff to a single channel (ch)
*******************************************************************************/
int start_pwm_count(uint32_t duty_cycle) {
	uint32_t *pruDRAM_32int_ptr = pru0DRAM_32int_ptr;
    uint32_t old_duty_cycle = pruDRAM_32int_ptr[0];
    uint32_t old_count_on = (100 - (100 - old_duty_cycle)) * CYCLES_PER_PERIOD / 100;
    uint32_t old_count_off = (100 - old_duty_cycle) * CYCLES_PER_PERIOD / 100;
    uint32_t new_count_on = (100 - (100 - duty_cycle)) * CYCLES_PER_PERIOD / 100;
    uint32_t new_count_off = (100 - duty_cycle) * CYCLES_PER_PERIOD / 100;
	
	printf("old:\n\tcountOn: %d, countOff: %d, count: %d\nnew:\n\tcountOn: %d, countOff: %d, count: %d\n", old_count_on, old_count_off, old_count_off + old_count_on, new_count_on, new_count_off, new_count_off + new_count_on);
	// write to PRU shared memory
	pruDRAM_32int_ptr[1] = PRU_TIMER_PASSCODE;
	pruDRAM_32int_ptr[0] = duty_cycle;	// On time
	return 0;
}

int main(int argc, char *argv[])
{
	uint32_t *pru;		// Points to start of PRU memory.
	int	fd;
	printf("Servo tester\n");
	
	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1) {
		printf ("ERROR: could not open /dev/mem.\n\n");
		return 1;
	}
	pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED) {
		printf ("ERROR: could not map memory.\n\n");
		return 1;
	}
	close(fd);
	printf ("Using /dev/mem.\n");
	
	pru0DRAM_32int_ptr =     pru + PRU0_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU0 memory
	pru1DRAM_32int_ptr =     pru + PRU1_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU1 memory
	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;	// Points to start of shared memory

    uint32_t desired_duty_cycle = 0;
    while (1) {
        printf("Enter a duty cycle: ");
        scanf("%d", &desired_duty_cycle);
        start_pwm_count(desired_duty_cycle);
    }
	
	if(munmap(pru, PRU_LEN)) {
		printf("munmap failed\n");
	} else {
		printf("munmap succeeded\n");
	}
}
