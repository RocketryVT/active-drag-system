#pragma once

#include <stdint.h>

//<sys/types.h> must be included before <inttypes.h> to fix WSL cmake compilation
#include <sys/types.h>
#include <inttypes.h>

#include "iim42653.hpp"
#include "adxl375.hpp"
#include "mmc5983ma.hpp"
#include "ms5607.hpp"
extern "C" {
#include "fix16.h"
}


#define PACKET_SIZE 64
#define PAD_SECONDS 2
#define LOG_RATE_HZ 100
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

    // 16 bytes MS5607 data
    uint32_t pressure :24; // 3
    int32_t altitude :24; // 3
    int32_t altitude_filt :32; // 4
    int32_t velocity_filt:32; // 4
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

    // 12 bytes controls information
    int32_t drag_force: 32; // 4
    int32_t apogee_prediction: 32; // 4
    int32_t desired_drag_force: 32; // 4

    uint8_t data0: 8;
} __attribute__((packed)) log_entry_t;

void print_log_entry(const uint8_t* entry);
