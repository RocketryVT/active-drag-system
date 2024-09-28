#include <stdio.h>
#include "pico/stdio.h"

#define PHASE_A_HIGH 0
#define PHASE_B_HIGH 0
#define PHASE_C_HIGH 0
#define PHASE_A_LOW 0
#define PHASE_B_LOW 0
#define PHASE_C_LOW 0

#define HALL_A 0
#define HALL_B 0
#define HALL_C 0

int main() {
    stdio_init_all();
    while (1) {
        tight_loop_contents();
    }
}
