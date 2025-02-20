#pragma once

#include <stdio.h>
#include <stdbool.h>

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "measure_pwm.pio.h"
#include "pico/time.h"

#define MICRO_SPI_CS        1
#define MICRO_SPI_TX        3
#define MICRO_SPI_RX        4
#define MICRO_SPI_SCLK      2
#define MICRO_MOTOR_FGOUT   6
#define MICRO_MOTOR_PWM     7
#define MICRO_MOTOR_BRAKE   8
#define MICRO_MOTOR_nSLEEP  9
#define MICRO_MOTOR_nFAULT 10
#define MICRO_MOTOR_DRVOFF 11

#define MOTOR_UPDATE_HZ 50

/*
 The SDI input data word is 16 bits long and consists of the following format:
 • 1 read or write bit, W (15)
 • 6 address bits, A      (14-9)
 • Parity bit, P          (8)
 • 8 data bits, D         (7-0)
 */

#define WRITE 0
#define READ 1

#define STATUS_REGISTER_COUNT 3
#define CONTROL_REGISTER_COUNT 10

// ADRESS DEFINES
#define IC_STATUS_REGISTER_ADDRESS 0x0
#define IC_STATUS_REGISTER_1_ADDRESS 0x1
#define IC_STATUS_REGISTER_2_ADDRESS 0x2
#define IC_CONTROL_REGISTER_1_ADDRESS 0x3
#define IC_CONTROL_REGISTER_2_ADDRESS 0x4
#define IC_CONTROL_REGISTER_3_ADDRESS 0x5
#define IC_CONTROL_REGISTER_4_ADDRESS 0x6
#define IC_CONTROL_REGISTER_5_ADDRESS 0x7
#define IC_CONTROL_REGISTER_6_ADDRESS 0x8
#define IC_CONTROL_REGISTER_7_ADDRESS 0x9
#define IC_CONTROL_REGISTER_8_ADDRESS 0xA
#define IC_CONTROL_REGISTER_9_ADDRESS 0xB
#define IC_CONTROL_REGISTER_10_ADDRESS 0xC

// IC_Status_Register
#define STATUS_REGISTER_NO_MOTOR_LOCK 0x0
#define STATUS_REGISTER_MOTOR_LOCK 0x1

#define STATUS_REGISTER_NO_BUCK_FAULT 0x0
#define STATUS_REGISTER_BUCK_FAULT 0x1

#define STATUS_REGISTER_NO_SPI_FAULT 0x0
#define STATUS_REGISTER_SPI_FAULT 0x1

#define STATUS_REGISTER_NO_OVERCURRENT 0x0
#define STATUS_REGISTER_OVERCURRENT 0x1

#define STATUS_REGISTER_POWER_ON_RESET_VM 0x0
#define STATUS_REGISTER_NO_POWER_ON_RESET 0x1

#define STATUS_REGISTER_NO_OVERVOLTAGE 0x0
#define STATUS_REGISTER_OVERVOLTAGE 0x1

#define STATUS_REGISTER_NO_OVERTEMPERATURE 0x0
#define STATUS_REGISTER_OVERTEMPERATURE 0x1

#define STATUS_REGISTER_NO_FAULT 0x0
#define STATUS_REGISTER_FAULT 0x1

// Masks to exclude bits marked as 'RESERVED' in datasheet
#define STATUS_REGISTER_MASK	 0x11111111
#define CONTROL_REGISTER_1_MASK  0x00000111
#define CONTROL_REGISTER_2_MASK  0x00111111
#define CONTROL_REGISTER_3_MASK  0x00011101
#define CONTROL_REGISTER_4_MASK  0x11111111
#define CONTROL_REGISTER_5_MASK  0x01001111
#define CONTROL_REGISTER_6_MASK  0x00011111
#define CONTROL_REGISTER_7_MASK  0x00011111
#define CONTROL_REGISTER_8_MASK  0x11011111
#define CONTROL_REGISTER_9_MASK  0x00000111
#define CONTROL_REGISTER_10_MASK 0x00011111

typedef union {
	struct {
		// This register is READ ONLY
		bool FAULT :1; // Device Fault
		bool OT :1; // Overtemperature Fault
		bool OVP :1; // Supply Overvoltage Protection Status
		bool NPOR :1; // Supply Power On Reset
		bool OCP :1; // Over Current Protection Status
		bool SPI_FLT :1; // SPI Fault
		bool BK_FLT :1; // Buck Fault
		bool MTR_LOCK :1; // Motor Lock
	} fields;
	uint8_t data;
} IC_Status_Register;

// IC_Status_Register1
#define STATUS_REGISTER_1_NO_OVERTEMPERATURE_WARNING 0x0
#define STATUS_REGISTER_1_OVERTEMPERATURE_WARNING 0x1

#define STATUS_REGISTER_1_NO_OVERTEMPERATURE_SHUTDOWN 0x0
#define STATUS_REGISTER_1_OVERTEMPERATURE_SHUTDOWN 0x1

#define STATUS_REGISTER_1_NO_OVERCURRENT_HIGHSIDE_OUTC 0x0
#define STATUS_REGISTER_1_OVERCURRENT_HIGHSIDE_OUTC 0x1

#define STATUS_REGISTER_1_NO_OVERCURRENT_LOWSIDE_OUTC 0x0
#define STATUS_REGISTER_1_OVERCURRENT_LOWSIDE_OUTC 0x1

#define STATUS_REGISTER_1_NO_OVERCURRENT_HIGHSIDE_OUTB 0x0
#define STATUS_REGISTER_1_OVERCURRENT_HIGHSIDE_OUTB 0x1

#define STATUS_REGISTER_1_NO_OVERCURRENT_LOWSIDE_OUTB 0x0
#define STATUS_REGISTER_1_OVERCURRENT_LOWSIDE_OUTB 0x1

#define STATUS_REGISTER_1_NO_OVERCURRENT_HIGHSIDE_OUTA 0x0
#define STATUS_REGISTER_1_OVERCURRENT_HIGHSIDE_OUTA 0x1

#define STATUS_REGISTER_1_NO_OVERCURRENT_LOWSIDE_OUTA 0x0
#define STATUS_REGISTER_1_OVERCURRENT_LOWSIDE_OUTA 0x1

typedef union {
	struct {
		// This register is READ ONLY
		bool OCP_LA :1; // Overcurrent on Low-side switch OUTA
		bool OCP_HA :1; // Overcurrent on High-side switch OUTA
		bool OCP_LB :1; // Overcurrent on Low-side switch OUTB
		bool OCP_HB :1; // Overcurrent on High-side switch OUTB
		bool OCL_LC :1; // Overcurrent on Low-side switch OUTC
		bool OCP_HC :1; // Overcurrent on High-side switch OUTC
		bool OTS :1; // Overtemperature Shutdown
		bool OTW :1; // Overtemperature Warning
	} fields;
	uint8_t data;
} IC_Status_Register1;

// IC_Status_Register2
#define STATUS_REGISTER_2_NO_OTP_ERROR 0
#define STATUS_REGISTER_2_OTP_ERROR 1

#define STATUS_REGISTER_2_NO_BUCK_OVERCURRENT 0x0
#define STATUS_REGISTER_2_BUCK_OVERCURRENT 0x1

#define STATUS_REGISTER_2_NO_BUCK_UNDERVOLTAGE 0x0
#define STATUS_REGISTER_2_BUCK_UNDERVOLTAGE 0x1

#define STATUS_REGISTER_2_NO_CHARGE_PUMP_UNDERVOLTAGE 0x0
#define STATUS_REGISTER_2_CHARGE_PUMP_UNDERVOLTAGE 0x1

#define STATUS_REGISTER_2_NO_SPI_PARITY_ERROR 0x0
#define STATUS_REGISTER_2_SPI_PARITY_ERROR 0x1

#define STATUS_REGISTER_2_NO_SCLK_FRAMING_ERROR 0x0
#define STATUS_REGISTER_2_SCLK_FRAMING_ERROR 0x1

#define STATUS_REGISTER_2_NO_SPI_ADDR_FAULT 0x0
#define STATUS_REGISTER_2_SPI_ADDR_FAULT 0x1

typedef union {
	struct {
		// This register is READ ONLY
		bool SPI_ADDR_FLT :1; //SPI Address Error
		bool SPI_SCLK_FLT :1; // SPI Clock Framing Error
		bool SPI_PARITY :1; // SPI Parity Error
		bool VCP_UV :1; // Charge Pump Undervoltage
		bool BUCK_UV :1; // Buck Regulator Undervoltage Status
		bool BUCK_OCP :1; // Buck Regulator Overcurrent Status
		bool OTP_ERR :1; // One Time Programmability Error
		bool RESERVED :1; // RESERVED
	} fields;
	uint8_t data;
} IC_Status_Register2;

// IC_Control_Register1
#define CONTROL_REGISTER_1_REG_LOCK_UNLOCK_ALL_REGISTERS 0x3
#define CONTROL_REGISTER_1_REG_LOCK_LOCK_ALL_REGISTERS 0x6

typedef union {
	struct {
		// Bit 7-3 Reserved
		uint8_t REG_LOCK :3; // Register Lock
		uint8_t RESERVED :5; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register1;

// IC_Control_Register2
#define CONTROL_REGISTER_2_SDO_MODE_SDO_IO_IN_OPEN_DRAIN_MODE 0x00
#define CONTROL_REGISTER_2_SDO_MODE_SDO_IO_IN_PUSH_PULL_MODE 0x01

#define CONTROL_REGISTER_2_SLEW_RATE_25_V_PER_US 0x00
#define CONTROL_REGISTER_2_SLEW_RATE_50_V_PER_US 0x01
#define CONTROL_REGISTER_2_SLEW_RATE_125_V_PER_US 0x02
#define CONTROL_REGISTER_2_SLEW_RATE_200_V_PER_US 0x03

#define CONTROL_REGISTER_2_PWM_MODE_ASYNCHRONOUS_RECTIFICATION_WITH_ANALOG_HALL 0x00
#define CONTROL_REGISTER_2_PWM_MODE_ASYNCHRONOUS_RECTIFICATION_WITH_DIGITAL_HALL 0x01
#define CONTROL_REGISTER_2_PWM_MODE_SYNCHRONOUS_RECTIFICATION_WITH_ANALOG_HALL 0x02
#define CONTROL_REGISTER_2_PWM_MODE_SYNCHRONOUS_RECTIFICATION_WITH_DIGITAL_HALL 0x3

#define CONTROL_REGISTER_2_NO_CLEAR_FAULT_CMD 0x00
#define CONTROL_REGISTER_2_CLEAR_FAULT_CMD 0x01

typedef union {
	struct {
		// Bit 7-6 Reserved
		bool CLR_FLAG :1; // Clear Fault
		uint8_t PWM_MODE :2; // Device Mode Selection
		uint8_t SLEW :2; // Slew Rrate Settings
		bool SDO_MODE :1; // SDO Mode Setting
		uint8_t RESERVED :2; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register2;

// IC_Control_Register3
#define CONTROL_REGISTER_3_PWM_100_DUTY_SEL_20KHZ 0x00
#define CONTROL_REGISTER_3_PWM_100_DUTY_SEL_40KHZ 0x01

#define CONTROL_REGISTER_3_OVERVOLTAGE_LVL_34V 0x00
#define CONTROL_REGISTER_3_OVERVOLTAGE_LVL_22V 0x01

#define CONTROL_REGISTER_3_OVERVOLTAGE_PROT_DISABLED 0x00
#define CONTROL_REGISTER_3_OVERVOLTAGE_PROT_ENABLED 0x01

#define CONTROL_REGISTER_3_OVERTEMPERATURE_nFAULT_DISABLED 0x00
#define CONTROL_REGISTER_3_OVERTEMPERATURE_nFAULT_ENABLED 0x01

typedef union {
	struct {
		// Bit 7-5 Reserved
		bool OTW_REP :1; // Overtemperature Waring Reporting Bit
		bool RESERVED_2 :1; // RESERVED
		bool OVP_EN :1; // Overvoltage Enable Bit
		bool OVP_SEL :1; // Overvoltage Level Bit
		bool PWM_100_DUTY_SEL :1; // frequency of PWM at 100% Duty Cycle
		uint8_t RESERVED :3; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register3;

// IC_Control_Register4
#define CONTROL_REGISTER_4_DRV_OFF_NO_ACTION 0x00
#define CONTROL_REGISTER_4_DRV_OFF_LOW_PWR_STANDBY 0x01

#define CONTROL_REGISTER_4_OVERCURRENT_CYCLE_BY_CYCLE_DISABLED 0x00
#define CONTROL_REGISTER_4_OVERCURRENT_CYCLE_BY_CYCLE_ENABLED 0x01

#define CONTROL_REGISTER_4_OCP_DEGLITCH_0_2US 0x00
#define CONTROL_REGISTER_4_OCP_DEGLITCH_0_6US 0x01
#define CONTROL_REGISTER_4_OCP_DEGLITCH_1_25US 0x02
#define CONTROL_REGISTER_4_OCP_DEGLITCH_1_6US 0x03

#define CONTROL_REGISTER_4_OCP_RETRY_5MS 0x00
#define CONTROL_REGISTER_4_OCP_RETRY_500MS 0x01

#define CONTROL_REGISTER_4_OCP_LVL_16A 0x00
#define CONTROL_REGISTER_4_OCP_LVL_24A 0x01

#define CONTROL_REGISTER_4_OCP_MODE_LATCHED_FAULT 0x00
#define CONTROL_REGISTER_4_OCP_MODE_AUTO_RETRY_FAULT 0x01
#define CONTROL_REGISTER_4_OCP_MODE_REPORT_ONLY 0x02
#define CONTROL_REGISTER_4_OCP_MODE_NOT_REPORTED 0x03

typedef union {
	struct {
		uint8_t OCP_MODE :2; // OCP Fault Options
		bool OCP_LVL :1; // Overcurrent Level Setting
		bool OCP_RETRY :1; // OCP Retry Time Settings
		uint8_t OCP_DEG :2; // OCP Deglitch Time Settings
		bool OCP_CBC :1; // OCP PWM Cycle Operation Bit
		bool DRV_OFF :1; // Driver Off Bit
	} fields;
	uint8_t data;
} IC_Control_Register4;

// IC_Control_Register5
#define CONTROL_REGISTER_5_ILIM_RECIR_BRAKE_MODE 0x00
#define CONTROL_REGISTER_5_ILIM_RECIR_COAST_MODE 0x01

#define CONTROL_REGISTER_5_AAR_DISABLED 0x00
#define CONTROL_REGISTER_5_AAR_ENABLED 0x01

#define CONTROL_REGISTER_5_ASR_DISABLED 0x00
#define CONTROL_REGISTER_5_ASR_ENABLED 0x01

#define CONTROL_REGISTER_5_CSA_GAIN_0_15V_PER_A 0x00
#define CONTROL_REGISTER_5_CSA_GAIN_0_3V_PER_A 0x01
#define CONTROL_REGISTER_5_CSA_GAIN_0_6V_PER_A 0x02
#define CONTROL_REGISTER_5_CSA_GAIN_1_2V_PER_A 0x03

typedef union {
	struct {
		uint8_t CSA_GAIN :2; // Current Sense Amplifiers Gain Settings
		uint8_t EN_ASR :1; // Active Synchronus Rectification Enable Bit
		bool EN_AAR :1; // Active Asynchronus Rectification Enable Bit
		bool RESERVED_2 :1; // RESERVED
		bool RESERVED_1 :1; // RESERVED
		bool ILIM_RECIR :1; // Current Limit Recicurcaltion Settings
		bool RESERVED :1; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register5;

// IC_Control_Register6
#define CONTROL_REGISTER_6_BUCK_POWER_SEQUENCING_ENABLED 0x00
#define CONTROL_REGISTER_6_BUCK_POWER_SEQUENCING_DISABLED 0x01

#define CONTROL_REGISTER_6_BUCK_CURR_LIM_600MA 0x00
#define CONTROL_REGISTER_6_BUCK_CURR_LIM_150MA 0x01

#define CONTROL_REGISTER_6_BUCK_VOLT_SEL_3_3V 0x00
#define CONTROL_REGISTER_6_BUCK_VOLT_SEL_5_0V 0x01
#define CONTROL_REGISTER_6_BUCK_VOLT_SEL_4_0V 0x02
#define CONTROL_REGISTER_6_BUCK_VOLT_SEL_5_7V 0x03

#define CONTROL_REGISTER_6_BUCK_ENABLED 0x00
#define CONTROL_REGISTER_6_BUCK_DISABLED 0x01

typedef union {
	struct {
		bool BUCK_DIS :1; // Buck Disable Bit
		uint8_t BUCK_SEL :2; // Buck Voltage Selection
		bool BUCK_CL :1; // Buck Current Limit Setting
		bool BUCK_PS_DIS :1; // Buck Power Sequencing Disable Bit
		bool RESERVED_1 :1; // RESERVED
		uint8_t RESERVED :2; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register6;

// IC_Control_Register7
#define CONTROL_REGISTER_7_HALL_HYS_5MV 0x00
#define CONTROL_REGISTER_7_HALL_HYS_50MV 0x01

#define CONTROL_REGISTER_7_BRAKE_MODE_BRAKING 0x00
#define CONTROL_REGISTER_7_BRAKE_MODE_COASTING 0x01

#define CONTROL_REGISTER_7_COAST_DISABLED 0x00
#define CONTROL_REGISTER_7_COAST_ENABLED 0x01

#define CONTROL_REGISTER_7_BRAKE_DISABLED 0x00
#define CONTROL_REGISTER_7_BRAKE_ENABLED 0x01

#define CONTROL_REGISTER_7_DIR_CW 0x00
#define CONTROL_REGISTER_7_DIR_CCW 0x01

typedef union {
	struct {
		bool DIR :1; // Direction Bit
		bool BRAKE :1; // Brake Bit
		bool COAST :1; // Coast Bit
		bool BRAKE_MODE :1; //Brake Mode Setting
		bool HALL_HYS :1; // Hall Comparator Hysteresis Settings
		uint8_t RESERVED :3; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register7;

// IC_Control_Register8
#define CONTROL_REGISTER_8_FGOUT_SEL_3X_COMM_FREQ 0x00
#define CONTROL_REGISTER_8_FGOUT_SEL_1X_COMM_FREQ 0x01
#define CONTROL_REGISTER_8_FGOUT_SEL_0_5X_COMM_FREQ 0x02
#define CONTROL_REGISTER_8_FGOUT_SEL_0_25X_COMM_FREQ 0x03

#define CONTROL_REGISTER_8_MTR_LOCK_RETRY_500MS 0x00
#define CONTROL_REGISTER_8_MTR_LOCK_RETRY_5000MS 0x01

#define CONTROL_REGISTER_8_MTR_LOCK_DETECT_TIME_300MS 0x00
#define CONTROL_REGISTER_8_MTR_LOCK_DETECT_TIME_500MS 0x01
#define CONTROL_REGISTER_8_MTR_LOCK_DETECT_TIME_1000MS 0x02
#define CONTROL_REGISTER_8_MTR_LOCK_DETECT_TIME_5000MS 0x03

#define CONTROL_REGISTER_8_MTR_LOCK_MODE_LATCHED_FAULT 0x00
#define CONTROL_REGISTER_8_MTR_LOCK_MODE_AUTO_RETRY_FAULT 0x01
#define CONTROL_REGISTER_8_MTR_LOCK_MODE_REPORT_ONLY 0x02
#define CONTROL_REGISTER_8_MTR_LOCK_MODE_NO_REPORT 0x03

typedef union {
	struct {
		uint8_t MTR_LOCK_MODE :2; // Motor Lock Fault Options
		uint8_t MTR_LOCK_TDET :2; // Motor Lock Detection Time Settings
		bool MTR_LOCK_RETRY :1; // Motor Lock Retry Time Settings
		bool RESERVED :1; //RESERVED
		uint8_t FGOUT_SEL :2; // Electrical Frequency Generation Output Mode Bits
	} fields;
	uint8_t data;
} IC_Control_Register8;

// IC_Control_Register9
#define CONTROL_REGISTER_9_ADVANCE_LVL_0DEG 0x00
#define CONTROL_REGISTER_9_ADVANCE_LVL_4DEG 0x01
#define CONTROL_REGISTER_9_ADVANCE_LVL_7DEG 0x02
#define CONTROL_REGISTER_9_ADVANCE_LVL_11DEG 0x03
#define CONTROL_REGISTER_9_ADVANCE_LVL_15DEG 0x04
#define CONTROL_REGISTER_9_ADVANCE_LVL_20DEG 0x05
#define CONTROL_REGISTER_9_ADVANCE_LVL_25DEG 0x06
#define CONTROL_REGISTER_9_ADVANCE_LVL_30DEG 0x07

typedef union {
	struct {
		uint8_t ADVANCED_LVL :3; // Phase Advance Settings
		uint8_t RESERVED :5; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register9;

// IC_Control_Register10
#define CONTROL_REGISTER_10_DLYCMP_EN_DISABLE 0x00
#define CONTROL_REGISTER_10_DLYCMP_EN_ENABLE 0x01

#define CONTROL_REGISTER_10_DLY_TARGET_0_0US 0x00
#define CONTROL_REGISTER_10_DLY_TARGET_0_4US 0x01
#define CONTROL_REGISTER_10_DLY_TARGET_0_6US 0x02
#define CONTROL_REGISTER_10_DLY_TARGET_0_8US 0x03
#define CONTROL_REGISTER_10_DLY_TARGET_1_0US 0x04
#define CONTROL_REGISTER_10_DLY_TARGET_1_2US 0x05
#define CONTROL_REGISTER_10_DLY_TARGET_1_4US 0x06
#define CONTROL_REGISTER_10_DLY_TARGET_1_6US 0x07
#define CONTROL_REGISTER_10_DLY_TARGET_1_8US 0x08
#define CONTROL_REGISTER_10_DLY_TARGET_2_0US 0x09
#define CONTROL_REGISTER_10_DLY_TARGET_2_2US 0x0A
#define CONTROL_REGISTER_10_DLY_TARGET_2_4US 0x0B
#define CONTROL_REGISTER_10_DLY_TARGET_2_6US 0x0C
#define CONTROL_REGISTER_10_DLY_TARGET_2_8US 0x0D
#define CONTROL_REGISTER_10_DLY_TARGET_3_0US 0x0E
#define CONTROL_REGISTER_10_DLY_TARGET_3_2US 0x0F



typedef union {
	struct {
		uint8_t DLY_TARGET :4; //Delay Target Driver Delay Compensation
		bool DLYCMP_EN :1; // Driver Delay Compensation enable
		uint8_t RESERVED :3; // RESERVED
	} fields;
	uint8_t data;
} IC_Control_Register10;

/* I actually fucking hate how SPI is implemented on RP2040 */
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(MICRO_SPI_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}


static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(MICRO_SPI_CS, 1);
    asm volatile("nop \n nop \n nop");
}

class mct8316z {
    public:
        mct8316z(spi_inst_t* inst);

        int8_t initialize();

        int8_t enable_motor();

        int8_t disable_motor();

        int8_t brake();

        int8_t set_speed(uint32_t speed);

        uint32_t get_speed() {
            return speed;
        }

        bool is_motor_enabled() {
            return motor_enabled;
        }

        bool is_motor_running() {
            return motor_running;
        }

        int8_t set_pwm(uint32_t freq, uint32_t duty);

        int8_t clear_faults();

        void update();

    private:
        uint8_t calc_parity(uint8_t value) {
            uint8_t parity = value;

            parity = parity ^ (value >> 1);
            parity = parity ^ (parity >> 2);
            parity = parity ^ (parity >> 4);

            return parity & 1;
        }
        void write_register(uint8_t addr, uint8_t data, uint8_t* buffer) {
            uint8_t rw = WRITE;

            uint8_t parity_1 = calc_parity((rw << 7) | (addr << 1));
            uint8_t parity_2 = calc_parity(data);

            uint8_t parity = parity_1 ^ parity_2;

            buffer[0] = (rw << 7) | (addr << 1) | (parity);  // remove read bit as this is a write
            buffer[1] = data;
            cs_select();
            spi_write_blocking(inst, buffer, 2);
            cs_deselect();
        };
        uint8_t read_register(uint8_t addr, uint8_t* buffer) {
            uint8_t rw = READ;

            uint8_t parity_1 = calc_parity((rw << 7) | (addr << 1));
            uint8_t parity_2 = calc_parity(0);

            uint8_t parity = parity_1 ^ parity_2;

            buffer[0] = (rw << 7) | (addr << 1) | (parity);
            buffer[1] = 0;

            cs_select();
            spi_write_blocking(spi0, buffer, 2);
            spi_read_blocking(spi0, 0, buffer, 2);
            cs_deselect();
            return buffer[1];

        };

        const uint32_t timing_interval_ms =  (1000 / TIMING_PULSE_FREQUENCY * TIMING_PULSE_RATIO);

        spi_inst_t* inst;

        uint8_t buffer[8];

        IC_Status_Register stat_reg;
        IC_Status_Register1 stat_reg_1;
        IC_Status_Register2 stat_reg_2;
        IC_Control_Register1 ctrl_reg_1;
        IC_Control_Register2 ctrl_reg_2;
        IC_Control_Register3 ctrl_reg_3;
        IC_Control_Register4 ctrl_reg_4;
        IC_Control_Register5 ctrl_reg_5;
        IC_Control_Register6 ctrl_reg_6;
        IC_Control_Register7 ctrl_reg_7;
        IC_Control_Register8 ctrl_reg_8;
        IC_Control_Register9 ctrl_reg_9;
        IC_Control_Register10 ctrl_reg_10;

        uint32_t speed_setpoint = 0;
        uint32_t speed = 0;

        bool motor_enabled = false;

        bool motor_running = false;

        repeating_timer_t motor_timer;

//        uint32_t speed_filtered = 0;
//        uint32_t speed_setpoint_filtered = 0;
};

