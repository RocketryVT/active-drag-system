#include <stdio.h>

#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "boards/pico.h"
#include "pico/stdio.h"
#include "pico/time.h"

#define MICRO_SPI0_CS   1
#define MICRO_SPI0_TX   3
#define MICRO_SPI0_RX   4
#define MICRO_SPI0_SCLK 2
#define MICRO_MOTOR_FGOUT   6
#define MICRO_MOTOR_PWM     7
#define MICRO_MOTOR_BRAKE   8
#define MICRO_MOTOR_nSLEEP  9
#define MICRO_MOTOR_nFAULT 10
#define MICRO_MOTOR_DRVOFF 11

#define HEART_RATE_HZ 5

#define READ_BIT 0x80


void heartbeat_core();
bool heartbeat_callback(repeating_timer_t *rt);

void process_cmd(char* buf, uint8_t len);
uint8_t cton(char c);

// static void read_registers(uint8_t reg, uint16_t *result);
// static void write_register(uint8_t reg, uint8_t data);
// static inline void cs_select();
// static inline void cs_deselect();


volatile uint8_t led_counter;
volatile bool motor_enabled = false;
repeating_timer_t heartbeat_timer;


int main() {
    stdio_init_all();

//    spi_init(spi0, 5000000);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

//    gpio_set_function(MICRO_SPI0_RX, GPIO_FUNC_SPI);
//    gpio_set_function(MICRO_SPI0_TX, GPIO_FUNC_SPI);
//    gpio_set_function(MICRO_SPI0_SCLK, GPIO_FUNC_SPI);
//    bi_decl(bi_3pins_with_func(MICRO_SPI0_RX, MICRO_SPI0_TX, MICRO_SPI0_SCLK, GPIO_FUNC_SPI));
//
//    gpio_init(MICRO_SPI0_CS);
//    gpio_set_dir(MICRO_SPI0_CS, GPIO_OUT);
//    gpio_put(MICRO_SPI0_CS, 1);
//    bi_decl(bi_1pin_with_name(MICRO_SPI0_CS, "SPI CS"));

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    multicore_launch_core1(heartbeat_core);

    char c;
    char buf[16] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t idx = 0;
    printf(">\t");
    while (1) {
        c = getchar_timeout_us(0);
        if (c != 255) {
            if (idx < 16) {
                if (c == 0x08 && idx > 0) { /* backspace */
                    idx--;
                    buf[idx] = 0;
                    printf("%c", 0x7f);
                }
                else if (c == 0x0D) { /* carriage return (enter) */
                    process_cmd(buf, idx);
                    for (uint8_t i = 16; i < 16; i++) {
                        buf[i] = 0;
                    }
                    idx = 0;
                    printf("\n>\t");
                }
                else {
                    buf[idx] = c;
                    idx++;
                }
                printf("%c", c);
            } else {
                printf("Overflow! press enter to reset buffer!\n");
                if (c == 0x0D) {
                    for (uint8_t i = 16; i < 16; i++) {
                        buf[i] = 0;
                    }
                    idx = 0;
                    printf("\n>\t");
                }
            }
        }
    }
}

void heartbeat_core() {
    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);

    while (1) {
        tight_loop_contents();
    }
}

bool heartbeat_callback(repeating_timer_t *rt) {
    const bool sequence[] = {true, false, true, false, false};
    const uint8_t sequence_length = 5;

    bool led_status = sequence[led_counter];
    gpio_put(PICO_DEFAULT_LED_PIN, led_status);
    led_counter++;
    led_counter %= sequence_length;
    return true;
}

void process_cmd(char* buf, uint8_t len) {
    if (len > 0) {
        int8_t result = -1;
        switch (buf[0]) {
            case 'r': {
                if (len >= 4) {
                    uint8_t msb = cton(buf[2]);
                    uint8_t lsb = cton(buf[3]);
                    if (msb <= 0xF && lsb <= 0xF ) {
                        uint8_t reg = (msb << 4) | lsb;
                        uint16_t result = 0;
                        // read_registers(reg, &result);
                        printf("\nREG 0x%02X: 0x%04X\n", reg, result);
                        result = 0;
                    }
                }
                break;
            }
            case 'w': {
                if (len >= 7) {
                    uint8_t msb_reg = cton(buf[2]);
                    uint8_t lsb_reg = cton(buf[3]);

                    uint8_t msb_dat = cton(buf[5]);
                    uint8_t lsb_dat = cton(buf[6]);

                    if (msb_reg <= 0xF && lsb_reg <= 0xF && msb_dat <= 0xF && lsb_dat <= 0xF) {
                        uint8_t reg = (msb_reg << 4) | lsb_reg;
                        uint8_t data = (msb_dat << 4) | lsb_dat;
                        // write_register(reg, data);
                        printf("\nWrote 0x%02X to 0x%02X\n", data, reg);
                        result = 0;
                    }
                }
                break;
            }
            case 'p': {
                if (len >= 7) {
                    uint8_t msb_freq = cton(buf[2]);
                    uint8_t lsb_freq = cton(buf[3]);

                    uint8_t msb_duty = cton(buf[5]);
                    uint8_t lsb_duty = cton(buf[6]);
                    if (msb_freq <= 0xF && lsb_freq <= 0xF && msb_duty <= 0xF && lsb_duty <= 0xF) {
                        uint8_t frequency = (msb_freq << 4) | lsb_freq;
                        uint8_t duty_cycle = (msb_duty << 4) | lsb_duty;
                        if (frequency > 200) frequency = 200;
                        if (duty_cycle > 100) duty_cycle = 100;
                        if (motor_enabled) {
                            printf("\nOperating motor at %d kHz with a %d percent duty cycle\n", frequency, duty_cycle);
                        } else {
                            printf("\nMotor is disabled! Enable motor with 'enable' to access this command!\n");
                        }
                        result = 0;
                    }
                    break;
                }
            }
            case 'd': {
                if (len >= 3 && buf[1] == 'i' && buf[2] == 'r') {
                    printf("\nchanging motor direction!\n");
                    result = 0;
                }
                break;
            }
            case 's': {
                printf("\nStopping motor!\n");
                result = 0;
                break;
            }
            case 'e': {
                if (len >= 6) {
                    if (buf[1] == 'n' && buf[2] == 'a' && buf[3] == 'b' && buf[4] == 'l' && buf[5] == 'e') {
                        printf("\nEnabling Motor!\n");
                        result = 0;
                    }
                }
                break;
            }
            default:
                break;
        }
        if (result < 0) {
            printf("Invalid command, try again!\n");
        }
    }
}

/* Returns 255 if char not 0-9 or A-F */
uint8_t cton(char c) {
    uint8_t result = 255;
    if (c >= 'A' && c <= 'F') {
        result = (c - 'A') + 0xA;
    } else if (c >= 'a' && c <= 'f') {
        result = (c - 'a') + 0xA;
    } else if (c >= '0' && c <= '9') {
        result = (c - '0');
    }
    return result;
}

///* I actually fucking hate how SPI is implemented on RP2040 */
//static inline void cs_select() {
//    asm volatile("nop \n nop \n nop");
//    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
//    asm volatile("nop \n nop \n nop");
//}
//
//static inline void cs_deselect() {
//    asm volatile("nop \n nop \n nop");
//    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
//    asm volatile("nop \n nop \n nop");
//}
//
//static void write_register(uint8_t reg, uint8_t data) {
//    uint8_t buf[2];
//    buf[0] = reg & 0x7f;  // remove read bit as this is a write
//    buf[1] = data;
//    cs_select();
//    spi_write_blocking(spi0, buf, 2);
//    cs_deselect();
//    sleep_ms(10);
//}
//
//static void read_registers(uint8_t reg, uint16_t *result) {
//    // For this particular device, we send the device the register we want to read
//    // first, then subsequently read from the device. The register is auto incrementing
//    // so we don't need to keep sending the register we want, just the first.
//    uint8_t buf[2] = {0, 0};
//    reg |= READ_BIT;
//    cs_select();
//    spi_write_blocking(spi0, &reg, 1);
//    sleep_ms(10);
//    spi_read_blocking(spi0, 0, buf, 2);
//    cs_deselect();
//    sleep_ms(10);
//    result = (((uint16_t) buf[0] << 8) | ((uint16_t) buf[1]));
//}
