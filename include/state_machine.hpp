#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

extern volatile float altitude = 0.0f;
extern volatile float prev_altitude = 0.0f;
extern volatile float velocity;
extern volatile state_t state;
extern volatile float threshold_altitude;
extern volatile float threshold_velocity;
extern volatile uint8_t deployment_percent;
extern volatile vector3f linear_acceleration;
extern volatile vector3f acceleration;
extern volatile quarternion abs_quaternion;
extern volatile vector3f velocity_vector;
extern volatile vector3f euler_angles;
extern volatile vector3f abs_lin_accel;
extern volatile vector3f prev_abs_lin_accel;
extern volatile vector3f rot_y_vec;
extern volatile vector3f vel_at_angle;
extern volatile vector3f accel_gravity;
extern volatile CALIB_STATUS calib_status;

#endif

