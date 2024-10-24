#include <stdio.h>

#include "boards/pico_w.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/time.h"

//Generate a 40kHz square wave with duty 0.5 on pin 21
int main() {
	stdio_init_all();
	
	//21   --> GPIO Selection, Pin 21 (Clock 0)
	//0x6  --> Input selection, CLK_SYS as input (125MHz)
	//3125 --> Clock Divider Int, 125MHz/40kHz = 3125
	clock_gpio_init(21, 0x6, 3125);

	while(1) {
		printf("Clock may or may not be running?");
		sleep_ms(1000);
	}
}
