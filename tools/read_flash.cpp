#include <stdio.h>
#include <inttypes.h>
#include "pico_logger.h"

#include "mid_imu.hpp"
#include "high_accel.hpp"
#include "magnetometer.hpp"
#include "altimeter.hpp"

#define PACKET_SIZE 43
#define PAD_SECONDS 8

MidIMU mid(i2c_default);
HighAccel high(i2c_default);
Magnetometer mag(i2c_default);

typedef enum {
    PAD = 0,
    BOOST,
    COAST,
    APOGEE,
    RECOVERY,
    END
} state_t;

typedef struct {
    // 11 bytes General time and state data
    uint64_t time_us: 64;
    state_t state: 4;
    uint16_t temperature_chip: 12;
    uint8_t deploy_percent: 8;

    // 7.25 bytes MS5607 data
    uint32_t pressure: 18;
    int32_t altitude: 24;
    int16_t temperature_alt: 16;

    // 12 bytes IMU data
    int16_t ax : 16;
    int16_t ay : 16;
    int16_t az : 16;
    int16_t gx : 16;
    int16_t gy : 16;
    int16_t gz : 16;

    // 6.75 bytes MAG data
    int32_t mag_x: 18;         // 18 bits (1.125 bytes) == 27.125
    int32_t mag_y: 18;         // 18 bits (1.125 bytes) == 28.25
    int32_t mag_z: 18;         // 18 bits (1.125 bytes) == 29.375

    // 6 bytes High G Accel data
    int16_t high_g_x : 16;
    int16_t high_g_y : 16;
    int16_t high_g_z : 16;
} __attribute__((packed)) log_entry_t;

void print_log_entry(const uint8_t* entry) {
    static bool first_call = true;

    if (first_call) {
        printf("time_us,state,temperature_chip_celsius,deployment_percent,pressure_mb,altitude_m,temperature_alt_celsius,mid_imu_ax,mid_imu_ay,mid_imu_az,mid_imu_gx,mid_imu_gy,mid_imu_gz,mag_x,mag_y,mag_z,high_g_x,high_g_y,high_g_z\r\n");
        first_call = false;
    }

    const log_entry_t* packet = reinterpret_cast<const log_entry_t *>(entry);

    printf("%" PRIu64 ",", packet->time_us);
    state_t state = (state_t) packet->state;
    switch (state) {
        case PAD:
            printf("PAD,");
            break;
        case BOOST:
            printf("BOOST,");
            break;
        case COAST:
            printf("COAST,");
            break;
        case APOGEE:
            printf("APOGEE,");
            break;
        case RECOVERY:
            printf("RECOVERY,");
            break;
        case END:
            printf("END,");
            break;
    }
    const float conversionFactor = 3.3f / (1 << 12);
    float tempC = 27.0f - (((float)(packet->temperature_chip) * conversionFactor) - 0.706f) / 0.001721f;
    printf("%4.2f,", tempC);
    printf("%d,", packet->deploy_percent);
    printf("%4.2f,", ((float) packet->pressure) / PRESSURE_SCALE_F);
    printf("%4.2f,", ((float) packet->altitude) / ALTITUDE_SCALE_F);
    printf("%4.2f,", ((float) packet->temperature_alt) / TEMPERATURE_SCALE_F);

    printf("%4.2f,", mid.scale_accel(packet->ax));
    printf("%4.2f,", mid.scale_accel(packet->ay));
    printf("%4.2f,", mid.scale_accel(packet->az));
    
    printf("%4.2f,", mid.scale_gyro(packet->gx));
    printf("%4.2f,", mid.scale_gyro(packet->gy));
    printf("%4.2f,", mid.scale_gyro(packet->gz));
    
    printf("%4.2f,", mag.scale(packet->mag_x));
    printf("%4.2f,", mag.scale(packet->mag_y));
    printf("%4.2f,", mag.scale(packet->mag_z));
    
    printf("%4.2f,", high.scale(packet->high_g_x));
    printf("%4.2f,", high.scale(packet->high_g_y));
    printf("%4.2f", high.scale(packet->high_g_z));
    printf("\r\n");
}

Logger logger(PACKET_SIZE, LOG_BASE_ADDR, &print_log_entry);

int main() {
    stdio_init_all();
    getchar();

    logger.initialize(false);

    // logger.erase_memory();
    logger.read_memory();
}
