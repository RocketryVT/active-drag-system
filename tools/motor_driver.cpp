#include <cstdint>
#include <stdio.h>

#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "boards/pico.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"

#include "measure_pwm.pio.h"

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

#define MICRO_MOTOR_POS_SENS_SDA 14
#define MICRO_MOTOR_POS_SENS_SCL 15
#define MICRO_MOTOR_POS_ADDR 0x06

#define HEART_RATE_HZ 5

#define READ_BIT 0x80


void heartbeat_core();
bool heartbeat_callback(repeating_timer_t *rt);

void process_cmd(char* buf, uint8_t len);
uint8_t cton(char c);

static uint8_t parity_calc(uint8_t value);
static void read_registers(uint8_t reg, uint16_t *result);
static void write_register(uint8_t reg, uint8_t data);
static inline void cs_select();
static inline void cs_deselect();

void set_pwm_pin(uint32_t pin, uint32_t freq, uint32_t duty_c); // duty_c between 0..10000

void timing_pulse_callback();

const uint32_t timing_interval_ms =  (1000 / TIMING_PULSE_FREQUENCY * TIMING_PULSE_RATIO);

volatile uint8_t led_counter;
volatile bool motor_enabled = false;
repeating_timer_t heartbeat_timer;

uint slice_num = 0;

int main() {
    stdio_init_all();

    spi_init(spi0, 5000000);
    i2c_init(i2c1, 400000);

    gpio_set_function(MICRO_MOTOR_POS_SENS_SCL, GPIO_FUNC_I2C);
    gpio_set_function(MICRO_MOTOR_POS_SENS_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(MICRO_MOTOR_POS_SENS_SCL);
    gpio_pull_up(MICRO_MOTOR_POS_SENS_SDA);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_init(MICRO_MOTOR_BRAKE);
    gpio_set_dir(MICRO_MOTOR_BRAKE, GPIO_OUT);

    gpio_init(MICRO_MOTOR_FGOUT);
    gpio_set_dir(MICRO_MOTOR_FGOUT, GPIO_IN);

    gpio_init(MICRO_MOTOR_nSLEEP);
    gpio_set_dir(MICRO_MOTOR_nSLEEP, GPIO_OUT);
    gpio_pull_up(MICRO_MOTOR_nSLEEP);
    gpio_put(MICRO_MOTOR_nSLEEP, 1);

    gpio_init(MICRO_MOTOR_nFAULT);
    gpio_set_dir(MICRO_MOTOR_nFAULT, GPIO_IN);

    gpio_init(MICRO_MOTOR_DRVOFF);
    gpio_set_dir(MICRO_MOTOR_DRVOFF, GPIO_OUT);
    gpio_pull_up(MICRO_MOTOR_DRVOFF);

    gpio_init(MICRO_MOTOR_PWM);

    gpio_set_function(MICRO_SPI0_RX, GPIO_FUNC_SPI);
    gpio_set_function(MICRO_SPI0_TX, GPIO_FUNC_SPI);
    gpio_set_function(MICRO_SPI0_SCLK, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(MICRO_SPI0_RX, MICRO_SPI0_TX, MICRO_SPI0_SCLK, GPIO_FUNC_SPI));

    gpio_init(MICRO_SPI0_CS);
    gpio_set_dir(MICRO_SPI0_CS, GPIO_OUT);
    gpio_put(MICRO_SPI0_CS, 1);
    bi_decl(bi_1pin_with_name(MICRO_SPI0_CS, "SPI CS"));

    gpio_init(TIMING_PULSE_PIN);
    gpio_set_dir(TIMING_PULSE_PIN, GPIO_OUT);
    gpio_set_function(TIMING_PULSE_PIN, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(TIMING_PULSE_PIN);
    pwm_config config = pwm_get_default_config();
    uint32_t duty_c = 32768;
    uint16_t wrap_val = UINT16_MAX;
    float clock_get_hz_var = ((float) clock_get_hz(clk_sys));
    float div = clock_get_hz_var / (((float) TIMING_PULSE_FREQUENCY) * UINT16_MAX);
    for (; (div < 1.0f); ) {
        duty_c /= ((div + 1) / div);
        wrap_val /= ((div + 1) / div);
        div += 1;
    }
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, wrap_val); 
    pwm_init(slice_num, &config, true); // start the pwm running according to the config
    pwm_set_gpio_level(TIMING_PULSE_PIN, 32768); //connect the pin to the pwm engine and set the on/off level. 

    PIO pio = pio0;
    uint offset = pio_add_program(pio0, &pulse_counter_pio_program);
    pulse_counter_pio_program_init(pio0, 0, offset, INPUT_PULSE_PIN);

    offset = pio_add_program(pio, &timing_pulse_pio_program);
    timing_pulse_pio_program_init(pio, 1, offset, TIMING_PULSE_PIN);
    pio_set_irq1_source_enabled(pio, pis_interrupt1, true);
    irq_add_shared_handler(PIO0_IRQ_1, timing_pulse_callback, 0);
    irq_set_enabled(PIO0_IRQ_1, true);

    pio_sm_restart(pio0, 0);
    pio_sm_restart(pio0, 1);

    pio_sm_set_enabled(pio0, 1, true);
    pio_sm_set_enabled(pio0, 0, true);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    multicore_launch_core1(heartbeat_core);


    getchar();
/******** INITIALIZE MOTOR DRIVER IC ************/
    printf("unlocking registers!\n");
    write_register(0x3, 0x3);

    printf("Disabling the buck converter!\n");
    write_register(0x8, 0b00010000);

    printf("Clearing latched fault bits!\n");
    write_register(0x4, 0b10000001);

    printf("Enabling brake mode!\n");
    write_register(0x9, 0b00000010);

    printf("Setting slew rate to 200 V per us!\n");
    printf("Enabling Hall PWM mode with Asynchronous rectification with digital hall!\n");
    write_register(0x4, 0b10011010);

    printf("Enabling active asynchronous rectification!\n");
    printf("Setting CSA gain to 1.2 V/A (artificial 1 A Curr Lim)\n");
    write_register(0x7, 0b00001011);

    printf("Setting Motor lock detection time to 500 ms!\n");
    printf("Setting motor lock to automatic retry!\n");
    write_register(0xA, 0b00000111);

    printf("Setting phase advance to 0 degrees!\n");
    write_register(0xB, 0b00000000);

    uint16_t result;

    read_registers(0x0, &result);

    printf("Status: 0x%02X\n", result);

    write_register(0x3, 0b00000110);

    uint8_t src = 0x3;
    uint8_t ibuf[2] = {0x0, 0x0};

    i2c_write_blocking(i2c1, MICRO_MOTOR_POS_ADDR, &src, 1, true);
    i2c_read_blocking(i2c1, MICRO_MOTOR_POS_ADDR, ibuf, 2, false);
    uint16_t raw_angle = ((((uint16_t) ibuf[0]) << 6) | (((uint16_t) ibuf[1]) << 2));
    float angular_pos = (((float) raw_angle) / 16384.0f) * 360.0f;
    printf("Raw angle: %04X; float angle: %4.2f\n", raw_angle, angular_pos);

/******** INITIALIZE MOTOR DRIVER IC ************/


    char c;
    char buf[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

        if (motor_enabled) {
            if (!pio_sm_is_rx_fifo_empty(pio, 1)) {
                if (pio_sm_get_blocking(pio, 1) == 1) {
                    if (!pio_sm_is_rx_fifo_empty(pio, 0)) {
                        uint32_t pulse_count = pio_sm_get_blocking(pio, 0);
                        pulse_count = (0x100000000 - pulse_count) & 0xFFFFFFFF;
                        printf("\nPIO Raw Count: %d, Freq: %d\n", pulse_count, (pulse_count * 1000 / timing_interval_ms));
                    }
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
                        read_registers(reg, &result);
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
                        write_register(reg, data);
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
                        uint32_t frequency = (msb_freq << 4) | lsb_freq;
                        uint32_t duty_cycle = (msb_duty << 4) | lsb_duty;
                        if (frequency > 200) frequency = 200;
                        if (duty_cycle > 100) duty_cycle = 100;

                        if (motor_enabled) {
                            printf("\nOperating motor at %d kHz with a %d percent duty cycle\n", frequency, duty_cycle);
                            frequency *= 1000;
                            duty_cycle = duty_cycle * (UINT16_MAX / 100);
                            set_pwm_pin(MICRO_MOTOR_PWM, frequency, duty_cycle);
                        } else {
                            printf("\nMotor is disabled! Enable motor with 'enable' to access this command!\n");
                        }
                        result = 0;
                    }
                    break;
                }
            }
            case 's': {
                printf("\nBraking!\n");
                gpio_put(MICRO_MOTOR_BRAKE, 1);
                sleep_ms(100);
                gpio_put(MICRO_MOTOR_DRVOFF, 1);
                set_pwm_pin(MICRO_MOTOR_PWM, 0, 0);
                printf("\nStopping motor!\n");
                motor_enabled = false;
                result = 0;
                break;
            }
            case 'e': {
                if (len >= 6) {
                    if (buf[1] == 'n' && buf[2] == 'a' && buf[3] == 'b' && buf[4] == 'l' && buf[5] == 'e') {
                        printf("\nEnabling Motor!\n");
                        motor_enabled = true;
                        gpio_put(MICRO_MOTOR_DRVOFF, 0);
                        gpio_put(MICRO_MOTOR_BRAKE, 0);
                        result = 0;
                    }
                }
                break;
            }
            case 't': {
                uint8_t src = 0x3;
                uint8_t ibuf[2] = {0x0, 0x0};
                i2c_write_blocking(i2c1, MICRO_MOTOR_POS_ADDR, &src, 1, true);
                i2c_read_blocking(i2c1, MICRO_MOTOR_POS_ADDR, ibuf, 2, false);
                uint16_t raw_angle = ((((uint16_t) ibuf[0]) << 6) | (((uint16_t) ibuf[1]) << 2));
                float angular_pos = (((float) raw_angle) / 16384.0f) * 360.0f;
                printf("Raw angle: %04X; float angle: %4.2f\n", raw_angle, angular_pos);
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

/* I actually fucking hate how SPI is implemented on RP2040 */
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(MICRO_SPI0_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(MICRO_SPI0_CS, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];

    uint8_t rw = 0;

    uint8_t parity_1 = parity_calc((rw << 7) | (reg << 1));
	uint8_t parity_2 = parity_calc(data);

    uint8_t parity = parity_1 ^ parity_2;

    buf[0] = (rw << 7) | (reg << 1) | (parity);  // remove read bit as this is a write
    buf[1] = data;
    cs_select();
    spi_write_blocking(spi0, buf, 2);
    cs_deselect();
    sleep_ms(10);
}

static void read_registers(uint8_t reg, uint16_t *result) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.
    uint8_t tx_buf[2] = {0, 0};
    uint8_t rx_buf[2] = {0, 0};
    uint8_t rw = 1;

    uint8_t parity_1 = parity_calc((rw << 7) | (reg << 1));
    uint8_t parity_2 = parity_calc(0);

    uint8_t parity = parity_1 ^ parity_2;

    tx_buf[0] = (rw << 7) | (reg << 1) | (parity);
    tx_buf[1] = 0;

    cs_select();
    spi_write_blocking(spi0, &reg, 1);
    spi_read_blocking(spi0, 0, rx_buf, 2);
    cs_deselect();
    printf("\nbuf[0]: %02X buf[1]: %02X\n", rx_buf[0], rx_buf[1]);
    sleep_ms(10);
    *result = ((((uint16_t) rx_buf[0]) << 8) | ((uint16_t) (rx_buf[1])));
}

static uint8_t parity_calc(uint8_t value) {
	// unsigned char because of 8 bit parity check
	// Initialize a variable to hold the parity value
	uint8_t parity = value;

	// XOR value with itself shifted right by 1 bit to get the parity of the first 2 bits
	parity = parity ^ (value >> 1);
	// XOR the result with itself shifted right by 2 bits to get the parity of the first 4 bits
	parity = parity ^ (parity >> 2);
	// XOR the result with itself shifted right by 4 bits to get the parity of all 8 bits
	parity = parity ^ (parity >> 4);

	// The parity is the least significant bit of the result
	return parity & 1;
}

void set_pwm_pin(uint32_t pin, uint32_t freq, uint32_t duty_c) { // duty_c between 0..65535
        printf("freq: %d\n", freq);
		gpio_set_function(pin, GPIO_FUNC_PWM);
		uint slice_num = pwm_gpio_to_slice_num(pin);
        uint16_t wrap_val = UINT16_MAX;
        pwm_config config = pwm_get_default_config();
        float clock_get_hz_var = ((float) clock_get_hz(clk_sys));
        printf("clock_get_hz: %4.2f\n", clock_get_hz_var);
		float div = clock_get_hz_var / (((float) freq) * UINT16_MAX);
        for (; (div < 1.0f); ) {
            duty_c /= ((div + 1) / div);
            wrap_val /= ((div + 1) / div);
            div += 1;
        }
        printf("div: %4.2f\n", div);
		pwm_config_set_clkdiv(&config, div);
		pwm_config_set_wrap(&config, wrap_val); 
		pwm_init(slice_num, &config, true); // start the pwm running according to the config
		pwm_set_gpio_level(pin, duty_c); //connect the pin to the pwm engine and set the on/off level. 
};

void timing_pulse_callback() {
    pwm_set_enabled(slice_num, false);
    pwm_set_counter(slice_num, 0);
    pwm_set_enabled(slice_num, true);
    pio_interrupt_clear(pio0, 1);
}
