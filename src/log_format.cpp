#include "log_format.hpp"

MidIMU* log_mid = NULL;
HighAccel* log_high = NULL;
Magnetometer* log_mag = NULL;

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
            printf("PAD");
            break;
        case BOOST:
            printf("BOOST");
            break;
        case COAST:
            printf("COAST");
            break;
        case APOGEE:
            printf("APOGEE");
            break;
        case RECOVERY:
            printf("RECOVERY");
            break;
        case END:
            printf("END");
            break;
    }
    printf(",");
    const float conversionFactor = 3.3f / (1 << 12);
    float tempC = 27.0f - (((float)(packet->temperature_chip) * conversionFactor) - 0.706f) / 0.001721f;
    printf("%4.2f,", tempC);
    printf("%d,", packet->deploy_percent);
    printf("%4.2f,", ((float) packet->pressure) / PRESSURE_SCALE_F);
    printf("%4.2f,", ((float) packet->altitude) / ALTITUDE_SCALE_F);
    printf("%4.2f,", ((float) packet->temperature_alt) / TEMPERATURE_SCALE_F);

    printf("%4.2f,", log_mid->scale_accel(packet->ax));
    printf("%4.2f,", log_mid->scale_accel(packet->ay));
    printf("%4.2f,", log_mid->scale_accel(packet->az));
    
    printf("%4.2f,", log_mid->scale_gyro(packet->gx));
    printf("%4.2f,", log_mid->scale_gyro(packet->gy));
    printf("%4.2f,", log_mid->scale_gyro(packet->gz));
    
    printf("%4.2f,", log_mag->scale(packet->mag_x));
    printf("%4.2f,", log_mag->scale(packet->mag_y));
    printf("%4.2f,", log_mag->scale(packet->mag_z));
    
    printf("%4.2f,", log_high->scale(packet->high_g_x));
    printf("%4.2f,", log_high->scale(packet->high_g_y));
    printf("%4.2f", log_high->scale(packet->high_g_z));
    printf("\r\n");
    stdio_flush();
    sleep_ms(10);
}

