#include "altimeter.hpp"
#include "hardware/gpio.h"

altimeter::altimeter(i2c_inst_t* inst, uint8_t addr) {
    this->inst = inst;
    this->addr = addr;
}

void altimeter::initialize() {
    // Select control register(0x26)
    // Active mode, OSR = 16, altimeter mode(0xB8)
    this->buffer[0] = 0x26;
    this->buffer[1] = 0x89;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
}

void altimeter::initialize(float threshold_altitude, uint8_t interrupt_pin, gpio_irq_callback_t callback) {
    this->initialize();

    // Below configures the interrupt for the first transition from PAD to BOOST
    // Initial Reading

    sleep_ms(1000);

    float altitude = 0.0f;

    while (altitude == 0.0f) {
        altitude = this->get_altitude_converted();
    }

    threshold_altitude += altitude; // 30 meters above ground

    // Select control register 3 (0x28)
    // Set bot interrupt pins to active low and enable internal pullups
    this->buffer[0] = 0x28;
    this->buffer[1] = 0x01;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select pressure target MSB register(0x16)
    // Set altitude target to 30 meters above ground altitude
    this->buffer[0] = 0x16;
    this->buffer[1] = (uint8_t) (((int16_t)(threshold_altitude)) >> 8);
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select pressure target LSB register(0x17)
    // Set altitude target to 30 meters above ground altitude
    this->buffer[0] = 0x17;
    this->buffer[1] = (uint8_t) (((int16_t)(threshold_altitude)));
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    this->buffer[0] = 0x29;
    this->buffer[1] = 0x08;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select interrupt this->bufferuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    this->buffer[0] = 0x2A;
    this->buffer[1] = 0x08;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    gpio_set_irq_enabled_with_callback(interrupt_pin, GPIO_IRQ_LEVEL_LOW, true, callback);
    // End of configuration of interrupt for first transition from PAD to BOOST
}

void altimeter::set_threshold_altitude(float threshold_altitude, uint8_t interrupt_pin, gpio_irq_callback_t callback) {
    float altitude = 0.0f;

    while (altitude == 0.0f) {
        altitude = get_altitude_converted();
    }

    threshold_altitude += altitude;

    // Select control register 3 (0x28)
    // Set bot interrupt pins to active low and enable internal pullups
    this->buffer[0] = 0x28;
    this->buffer[1] = 0x01;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select pressure target MSB register(0x16)
    // Set altitude target to 30 meters above ground altitude
    this->buffer[0] = 0x16;
    this->buffer[1] = (uint8_t) (((int16_t)(threshold_altitude)) >> 8);
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select pressure target LSB register(0x17)
    // Set altitude target to provided threshold altitude
    this->buffer[0] = 0x17;
    this->buffer[1] = (uint8_t) (((int16_t)(threshold_altitude)));
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    this->buffer[0] = 0x29;
    this->buffer[1] = 0x08;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select interrupt this->bufferuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    this->buffer[0] = 0x2A;
    this->buffer[1] = 0x08;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    gpio_set_irq_enabled_with_callback(interrupt_pin, GPIO_IRQ_LEVEL_LOW, true, callback);
    // End of configuration of interrupt for first transition from PAD to BOOST
}

void altimeter::unset_threshold_altitude(uint8_t interrupt_pin) {
    gpio_set_irq_enabled_with_callback(interrupt_pin, GPIO_IRQ_LEVEL_LOW, false, NULL);

    // Select interrupt enable register (0x29)
    // Set interrupt enabled for altitude threshold(0x08)
    this->buffer[0] = 0x29;
    this->buffer[1] = 0x00;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);

    // Select interrupt configuration register (0x2A)
    // Set interrupt enabled for altitude threshold to route to INT1 pin(0x08)
    this->buffer[0] = 0x2A;
    this->buffer[1] = 0x00;
    i2c_write_blocking(this->inst, this->addr, this->buffer, 2, true);
}

float altimeter::get_altitude_converted() {
    uint8_t reg = 0x01;
    i2c_write_blocking(this->inst, this->addr, &reg, 1, true);
    i2c_read_blocking(this->inst, this->addr, this->altitude_buffer, 4, false);
    // Exactly how MPL3115A2 datasheet says to retrieve altitude
    float altitude = (float) ((int16_t) ((this->altitude_buffer[0] << 8) | this->altitude_buffer[1])) + (float) (this->altitude_buffer[2] >> 4) * 0.0625;
    return altitude;
}

void altimeter::get_altitude_raw(uint8_t* buffer) {
    uint8_t reg = 0x01;
    i2c_write_blocking(this->inst, this->addr, &reg, 1, true);
    i2c_read_blocking(this->inst, this->addr, buffer, 3, false);
}

uint32_t altimeter::expose_buffer(uint8_t** buffer) {
    *buffer = this->altitude_buffer;
    return sizeof(this->altitude_buffer);
}

