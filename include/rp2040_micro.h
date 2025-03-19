// -----------------------------------------------------
// NOTE: THIS HEADER IS ALSO INCLUDED BY ASSEMBLER SO
//       SHOULD ONLY CONSIST OF PREPROCESSOR DIRECTIVES
// -----------------------------------------------------


#ifndef _BOARDS_RP2040_MICRO_H
#define _BOARDS_RP2040_MICRO_H

// For board detection
#define RP2040_MICRO_H

// --- UART ---
#ifndef PICO_DEFAULT_UART
#define PICO_DEFAULT_UART 0
#endif
#ifndef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 12
#endif
#ifndef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 13
#endif

// --- LED ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif
// no PICO_DEFAULT_WS2812_PIN

// --- I2C ---
#ifndef PICO_DEFAULT_I2C
#define PICO_DEFAULT_I2C 0
#endif
#ifndef PICO_DEFAULT_I2C_SDA_PIN
#define PICO_DEFAULT_I2C_SDA_PIN 20
#endif
#ifndef PICO_DEFAULT_I2C_SCL_PIN
#define PICO_DEFAULT_I2C_SCL_PIN 21
#endif

// --- SPI ---
#ifndef PICO_DEFAULT_SPI
#define PICO_DEFAULT_SPI 0
#endif
#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 18
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 19
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 16
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 17
#endif

// --- MISC ---
#ifndef MICRO_DEFAULT_PWM
#define MICRO_DEFAULT_PWM 7
#endif

#ifndef MICRO_DEFAULT_SERVO_ENABLE
#define MICRO_DEFAULT_SERVO_ENABLE 4
#endif

#ifndef MICRO_DEFAULT_CLK_OUTPUT
#define MICRO_DEFAULT_CLK_OUTPUT 23
#endif

#ifndef MICRO_IMU_INTTERUPT
#define MICRO_IMU_INTERRUPT 22
#endif

#ifndef MICRO_ACCEL_INTERRUPT_0
#define MICRO_ACCEL_INTERRUPT_0 19
#endif

#ifndef MICRO_ACCEL_INTERRUPT_1
#define MICRO_ACCEL_INTERRUPT_1 18
#endif

#ifndef MICRO_MAG_INTERRUPT
#define MICRO_MAG_INTERRUPT 17
#endif

#ifndef MICRO_BUZZER
#define MICRO_BUZZER 0
#endif

// --- FLASH ---

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif

#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif

// Drive high to force power supply into PWM mode (lower ripple on 3V3 at light loads)
#define PICO_SMPS_MODE_PIN 23

// All boards have B1 RP2040
#ifndef PICO_RP2040_B0_SUPPORTED 
#define PICO_RP2040_B0_SUPPORTED  0
#endif

#endif
