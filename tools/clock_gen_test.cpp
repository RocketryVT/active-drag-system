#include <stdio.h>

#include "boards/pico_w.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/time.h"

int main() {
	stdio_init_all();
	
	//21   --> GPIO Selection, Pin 21 (Clock 0)
	//0x6  --> Input selection, CLK_SYS as input (12MHz)
	//3125 --> Clock Divider Int, 125MHz/32kHz = 3906.25
	clock_gpio_init(21, 0x6, 3125);

	while(1) {
		printf("Clock may or may not be running?");
		sleep_ms(1000);
	}
}
