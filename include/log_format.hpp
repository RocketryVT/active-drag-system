#pragma once

#include <stdint.h>
#include <inttypes.h>

#include "mid_imu.hpp"
#include "high_accel.hpp"
#include "magnetometer.hpp"
#include "altimeter.hpp"


#define PACKET_SIZE 43
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

void print_log_entry(const uint8_t* entry);
