#pragma once

#include <stdint.h>
#include <inttypes.h>

#include "iim42653.hpp"
#include "adxl375.hpp"
#include "mmc5983ma.hpp"
#include "ms5607.hpp"


#define PACKET_SIZE 64
#define PAD_SECONDS 8
#define LOG_RATE_HZ 50
#define PAD_BUFFER_SIZE (PACKET_SIZE * PAD_SECONDS * LOG_RATE_HZ)

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
    uint64_t time_us :64;
    state_t state :4;
    uint16_t temperature_chip :12;
    uint8_t deploy_percent :8;

    // 8 bytes MS5607 data
    uint32_t pressure :24; // 3
    int32_t altitude :24; // 3
    int16_t temperature_alt :16; // 2

    // 12 bytes IMU data
    int16_t ax :16;
    int16_t ay :16;
    int16_t az :16;
    int16_t gx :16;
    int16_t gy :16;
    int16_t gz :16;

    // 6 bytes MAG data
    int16_t mag_x :16;
    int16_t mag_y :16;
    int16_t mag_z :16;

    // 6 bytes High G Accel data
    int16_t high_g_x :16;
    int16_t high_g_y :16;
    int16_t high_g_z :16;

    uint64_t data0: 64;
    uint64_t data1: 64;
    uint32_t data2: 32;
    uint8_t data3: 8;
} __attribute__((packed)) log_entry_t;

void print_log_entry(const uint8_t* entry);
