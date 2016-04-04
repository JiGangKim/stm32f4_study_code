#include "variable.h"

MOTOR_Typedef motor_right;
MOTOR_Typedef motor_left;

ENCODER_TypeDef encoder_right;
ENCODER_TypeDef encoder_left;

uint8_t mpu6050_buffer[14];
int16_t mpu6050_sense[7];

float yaw, pitch, roll;
int yaw_deg, pitch_deg, roll_deg;
float gx, gy, gz, ax, ay, az;
float gx_offset = 0;
float gy_offset = 0;
float gz_offset = 0;
float gyro_scale = 2.0f*PI*(250.0f/360.0f)*(1.0f/32767.0f);
float accel_scale = 2.0f*9.79f*(1.0f/32767.0f);

uint8_t flag_run;

SEGWAY_Typedef segway;

