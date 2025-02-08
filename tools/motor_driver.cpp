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

bool heartbeat_callback(repeating_timer_t *rt);

volatile uint8_t led_counter;
repeating_timer_t heartbeat_timer;

/* I actually fucking hate how SPI is implemented on RP2040 */
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
    asm volatile("nop \n nop \n nop");
}

static void write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg & 0x7f;  // remove read bit as this is a write
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
    uint8_t buf[2] = {0, 0};
    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(spi0, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(spi0, 0, buf, 2);
    cs_deselect();
    sleep_ms(10);
    result = (((uint16_t) buf[0] << 8) | ((uint16_t) buf[1]));
}

void heartbeat_core() {
    add_repeating_timer_us(-1000000 / HEART_RATE_HZ,  &heartbeat_callback, NULL, &heartbeat_timer);

    while (1) {
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();

    spi_init(spi0, 5000000);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_set_function(MICRO_SPI0_RX, GPIO_FUNC_SPI);
    gpio_set_function(MICRO_SPI0_TX, GPIO_FUNC_SPI);
    gpio_set_function(MICRO_SPI0_SCLK, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(MICRO_SPI0_RX, MICRO_SPI0_TX, MICRO_SPI0_SCLK, GPIO_FUNC_SPI));

    gpio_init(MICRO_SPI0_CS);
    gpio_set_dir(MICRO_SPI0_CS, GPIO_OUT);
    gpio_put(MICRO_SPI0_CS, 1);
    bi_decl(bi_1pin_with_name(MICRO_SPI0_CS, "SPI CS"));

    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    multicore_launch_core1(heartbeat_core);

    char c;
    while (1) {
        c = getchar();
        switch (c) {
            case 'r':
                break;
            case 'w':
                break;
            case 'p':

                break;
            default:
                break;
        }
        c = 0;
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
