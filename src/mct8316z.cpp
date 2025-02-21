#include "mct8316z.hpp"
#include "fixed_point.h"

static volatile uint8_t timing_pulse_slice_num = 0;

static void timing_pulse_callback() {
    pwm_set_enabled(timing_pulse_slice_num, false);
    pwm_set_counter(timing_pulse_slice_num, 0);
    pwm_set_enabled(timing_pulse_slice_num, true);
    pio_interrupt_clear(pio0, 1);
}

mct8316z::mct8316z(spi_inst_t* inst) {
    this->inst = inst;

    spi_init(this->inst, 5000000);

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
    gpio_set_dir(MICRO_MOTOR_PWM, GPIO_OUT);
    gpio_set_function(MICRO_MOTOR_PWM, GPIO_FUNC_PWM);

    gpio_set_function(MICRO_SPI_RX, GPIO_FUNC_SPI);
    gpio_pull_up(MICRO_SPI_RX);
    gpio_set_function(MICRO_SPI_TX, GPIO_FUNC_SPI);
    gpio_set_function(MICRO_SPI_SCLK, GPIO_FUNC_SPI);

    gpio_init(MICRO_SPI_CS);
    gpio_set_dir(MICRO_SPI_CS, GPIO_OUT);
    gpio_put(MICRO_SPI_CS, 1);

    gpio_init(TIMING_PULSE_PIN);
    gpio_set_dir(TIMING_PULSE_PIN, GPIO_OUT);
    gpio_set_function(TIMING_PULSE_PIN, GPIO_FUNC_PWM);

}

int8_t mct8316z::initialize() {
    timing_pulse_slice_num = pwm_gpio_to_slice_num(TIMING_PULSE_PIN);
    pwm_config config = pwm_get_default_config();
    uint32_t duty_cycle = (UINT16_MAX * 50) / 100;
    uint16_t wrap_value = UINT16_MAX;
    float clk_hz = ((float) clock_get_hz(clk_sys));
    float clk_div = clk_hz / (((float) TIMING_PULSE_FREQUENCY) * UINT16_MAX);
    for (; (clk_div < 1.0f); ) {
        duty_cycle /= ((clk_div + 1) / clk_div);
        wrap_value /= ((clk_div + 1) / clk_div);
        clk_div += 1;
    }
    pwm_config_set_clkdiv(&config, clk_div);
    pwm_config_set_wrap(&config, wrap_value); 
    pwm_init(timing_pulse_slice_num, &config, true);
    pwm_set_gpio_level(TIMING_PULSE_PIN, duty_cycle);

    PIO pio = pio0;
    uint offset = pio_add_program(pio, &pulse_counter_pio_program);
    pulse_counter_pio_program_init(pio, 0, offset, INPUT_PULSE_PIN);

    offset = pio_add_program(pio, &timing_pulse_pio_program);
    timing_pulse_pio_program_init(pio, 1, offset, TIMING_PULSE_PIN);
    pio_set_irq1_source_enabled(pio, pis_interrupt1, true);
    irq_add_shared_handler(PIO0_IRQ_1, timing_pulse_callback, 0);
    irq_set_enabled(PIO0_IRQ_1, true);

    pio_sm_restart(pio, 0);
    pio_sm_restart(pio, 1);

    uint8_t slice_num = pwm_gpio_to_slice_num(MICRO_MOTOR_PWM);
    config = pwm_get_default_config();
    pwm_config_set_clkdiv_int_frac(&config, 1, 4); // 1 MHz with 5000 (MOTOR_PWM_WRAP) wrap value = 20 kHz PWM
    pwm_config_set_wrap(&config, MOTOR_PWM_WRAP); 
    pwm_init(slice_num, &config, true);

    printf("Unlocking MCT8316Z Registers...\n");
    this->ctrl_reg_1.fields.REG_LOCK = 0b011;
    printf("\nREG_LOCK_DATA: %02X\n", this->ctrl_reg_1.data);
    write_register(IC_CONTROL_REGISTER_1_ADDRESS, (this->ctrl_reg_1.data & CONTROL_REGISTER_1_MASK), this->buffer);

    printf("Clearing latched fault bits...\n");
    this->ctrl_reg_2.fields.CLR_FLAG = CONTROL_REGISTER_2_CLEAR_FAULT_CMD;
    write_register(IC_CONTROL_REGISTER_2_ADDRESS, ((this->ctrl_reg_2.data & CONTROL_REGISTER_2_MASK) | CONTROL_REGISTER_2_RESERVED), this->buffer);

    printf("Enabling push-pull mode for SDO...\n");
    printf("Setting 200 V/us slew rate...\n");
    printf("Setting PWM mode to asynchronous rectification with digital hall...\n");
//    this->ctrl_reg_2.fields.SDO_MODE = CONTROL_REGISTER_2_SDO_MODE_SDO_IO_IN_PUSH_PULL_MODE;
    this->ctrl_reg_2.fields.SLEW = CONTROL_REGISTER_2_SLEW_RATE_200_V_PER_US;
    this->ctrl_reg_2.fields.PWM_MODE = CONTROL_REGISTER_2_PWM_MODE_ASYNCHRONOUS_RECTIFICATION_WITH_DIGITAL_HALL;
    write_register(IC_CONTROL_REGISTER_2_ADDRESS, (this->ctrl_reg_2.data & CONTROL_REGISTER_2_MASK), this->buffer);

    printf("Disabling the buck converter...\n");
    this->ctrl_reg_6.fields.BUCK_DIS = CONTROL_REGISTER_6_BUCK_DISABLED;
    write_register(IC_CONTROL_REGISTER_6_ADDRESS, (this->ctrl_reg_6.data & CONTROL_REGISTER_6_MASK), this->buffer);

    printf("Enabling brake mode...\n");
    printf("Counter clockwise direction...\n");
    this->ctrl_reg_7.fields.BRAKE_MODE = CONTROL_REGISTER_7_BRAKE_MODE_BRAKING;
    this->ctrl_reg_7.fields.DIR = CONTROL_REGISTER_7_DIR_CCW;
    write_register(IC_CONTROL_REGISTER_7_ADDRESS, (this->ctrl_reg_7.data & CONTROL_REGISTER_7_MASK), this->buffer);

    printf("Setting CSA gain to 0.6 V/A (artificial 2 A current limit)...\n");
    printf("Enabling active asynchronous rectification...\n");
    this->ctrl_reg_5.fields.CSA_GAIN = CONTROL_REGISTER_5_CSA_GAIN_0_6V_PER_A;
    this->ctrl_reg_5.fields.EN_AAR = CONTROL_REGISTER_5_ASR_ENABLED;
    write_register(IC_CONTROL_REGISTER_5_ADDRESS, (this->ctrl_reg_5.data & CONTROL_REGISTER_5_MASK), this->buffer);

    printf("Setting motor lock detection time to 500 ms...\n");
    printf("Setting motor lock response to automatic retry...\n");
    printf("Setting FGOUT signal to communicate 3x the commutation frequency...\n");
    this->ctrl_reg_8.fields.MTR_LOCK_MODE = CONTROL_REGISTER_8_MTR_LOCK_MODE_AUTO_RETRY_FAULT;
    this->ctrl_reg_8.fields.MTR_LOCK_TDET = CONTROL_REGISTER_8_MTR_LOCK_DETECT_TIME_500MS;
    this->ctrl_reg_8.fields.FGOUT_SEL = CONTROL_REGISTER_8_FGOUT_SEL_3X_COMM_FREQ;
    write_register(IC_CONTROL_REGISTER_8_ADDRESS, (this->ctrl_reg_8.data & CONTROL_REGISTER_8_MASK), this->buffer);

    printf("Setting phase advance to 0 degrees...\n");
    this->ctrl_reg_9.fields.ADVANCED_LVL = CONTROL_REGISTER_9_ADVANCE_LVL_0DEG;
    write_register(IC_CONTROL_REGISTER_9_ADDRESS, (this->ctrl_reg_9.data & CONTROL_REGISTER_9_MASK), this->buffer);


    this->stat_reg.data = read_register(IC_STATUS_REGISTER_ADDRESS, this->buffer);

    printf("Locking MCT8316Z Registers...\n");
    this->ctrl_reg_1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_LOCK_ALL_REGISTERS;
    write_register(IC_CONTROL_REGISTER_1_ADDRESS, (this->ctrl_reg_1.data & CONTROL_REGISTER_1_MASK), this->buffer);

    gpio_put(MICRO_MOTOR_DRVOFF, 1);
    gpio_put(MICRO_MOTOR_BRAKE, 1);


    return 0;
}

int8_t mct8316z::enable_motor() {
    printf("\nEnabling Motor!\n");
    gpio_put(MICRO_MOTOR_DRVOFF, 0);
    gpio_put(MICRO_MOTOR_BRAKE, 0);
    pio_sm_set_enabled(pio0, 1, true);
    pio_sm_set_enabled(pio0, 0, true);
    this->motor_enabled = true;
    return 0;
}

int8_t mct8316z::disable_motor() {
    printf("\nDisabling motor!\n");
    if (motor_enabled) {
        if (motor_running) {
            brake();
        }
        gpio_put(MICRO_MOTOR_DRVOFF, 1);
        this->motor_enabled = false;
    }
    return 0;
}

int8_t mct8316z::brake() {
    printf("\nBraking!\n");
    set_pwm(0);
    gpio_put(MICRO_MOTOR_BRAKE, 1);
    pio_sm_set_enabled(pio0, 0, false);
    pio_sm_set_enabled(pio0, 1, false);
    pio_sm_restart(pio0, 0);
    pio_sm_restart(pio0, 1);
    this->speed_setpoint = 0;
    this->speed_cmd = 0;
    this->motor_running = false;
    return 0;
}

int8_t mct8316z::set_speed(int16_t cmd) {
    int8_t result = -1;
    if (motor_enabled) {
        if (cmd > MOTOR_MAX_SPEED) {
            cmd = MOTOR_MAX_SPEED;
        } else if (cmd < 500) {
            cmd = 0;
        }
        this->speed_setpoint = cmd;
        result = 0;
    } else {
        printf("Enable the motor prior to setting a speed!\n");
    }
    return result;
}

int8_t mct8316z::set_pwm(int8_t duty_cycle) {
    int8_t result = -1;
    if (duty_cycle > 100) duty_cycle = 100;
    if (duty_cycle < 0) duty_cycle = 0;
    this->duty = duty_cycle;
    if (motor_enabled) {
        uint16_t duty_scaled = (uint16_t) ((((uint32_t) duty) * ((uint32_t) MOTOR_PWM_WRAP)) / ((uint32_t) 100));
        pwm_set_gpio_level(MICRO_MOTOR_PWM, duty_scaled);
        result = 0;
    } else {
        printf("Enable the motor prior to setting a PWM!\n");
    }
    return result;
};

int8_t mct8316z::clear_faults() {
    int8_t result = -1;
    if (!motor_running && !motor_enabled) {
        printf("Unlocking MCT8316Z Registers...\n");
        this->ctrl_reg_1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_UNLOCK_ALL_REGISTERS;
        write_register(IC_CONTROL_REGISTER_1_ADDRESS, (this->ctrl_reg_1.data & CONTROL_REGISTER_1_MASK), this->buffer);

        printf("Clearing latched fault bits...\n");
        this->ctrl_reg_2.fields.CLR_FLAG = CONTROL_REGISTER_2_CLEAR_FAULT_CMD;
        write_register(IC_CONTROL_REGISTER_2_ADDRESS, (this->ctrl_reg_2.data & CONTROL_REGISTER_2_MASK), this->buffer);

        printf("Locking MCT8316Z Registers...\n");
        this->ctrl_reg_1.fields.REG_LOCK = CONTROL_REGISTER_1_REG_LOCK_LOCK_ALL_REGISTERS;
        write_register(IC_CONTROL_REGISTER_1_ADDRESS, (this->ctrl_reg_1.data & CONTROL_REGISTER_1_MASK), this->buffer);
        result = 0;
    } else {
        printf("Disable the motor prior to writing to IC registers!\n");
    }
    return result;
}

void mct8316z::update() {
    if (motor_enabled) {
        if (!pio_sm_is_rx_fifo_empty(pio0, 1)) {
            if (pio_sm_get_blocking(pio0, 1) == 1) {
                if (!pio_sm_is_rx_fifo_empty(pio0, 0)) {
                    uint32_t pulse_count = pio_sm_get_blocking(pio0, 0);
                    pulse_count = (0x100000000 - pulse_count) & 0xFFFFFFFF;
                    if (pulse_count > 10) {
                        int16_t commutation_freq_3x = (int16_t) (((uint32_t) pulse_count) * ((uint32_t) 1000) / ((uint32_t) timing_interval_ms));
                        speed = ((commutation_freq_3x / 3) * 60);
                        speed_filtered = ((int16_t) speed_flt.update(((s1x14) speed)));
                        motor_running = true;
                    } else {
                        speed = 0;
                        speed_filtered = ((int16_t) speed_flt.update(((s1x14) speed)));
                        if ((speed_filtered == 0) && (duty != 0)) {
                            printf("Something seems wrong here...\n");
                        }
                        motor_running = false;
                    }
                }
            }
        }
                                                                                                    // 50 Hz
        speed_cmd = ((int16_t) speed_pid.update(((s1x14) speed_setpoint), ((s1x14) speed_filtered), float_to_s1x14(0.02f)));
        // printf("speed_cmd: %d\n", speed_cmd);
        if (speed_cmd > MOTOR_MAX_SPEED) {
            speed_cmd = MOTOR_MAX_SPEED;
        } else if (speed_cmd < 0) {
            speed_cmd = 0;
        }
        duty = (int8_t) (((((int32_t) speed_cmd) * ((int32_t) 100)) / ((int32_t) SUPPLY_VOLTAGE)) / ((int32_t) MOTOR_KV));
        set_pwm(duty);
    } else {
        speed = 0;
        speed_filtered = ((int16_t) speed_flt.update(((s1x14) speed)));
                                // 50 Hz
        speed_pid.update(0, 0, float_to_s1x14(0.02f));
    }

//    read_register(0, buffer);
//    printf("%02X%02X\n", buffer[0], buffer[1]);

//    printf("%d,%d,%d,%d\n", speed, speed_filtered, speed_cmd, speed_setpoint);
}
