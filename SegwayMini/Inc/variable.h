#ifndef VARIABLE_H_
#define VARIABLE_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "encoder.h"
#include "segway.h"

extern MOTOR_Typedef motor_right;
extern MOTOR_Typedef motor_left;

extern ENCODER_TypeDef encoder_right;
extern ENCODER_TypeDef encoder_left;

extern uint8_t mpu6050_buffer[];
extern int16_t mpu6050_sense[];

extern float yaw, pitch, roll;
extern int yaw_deg, pitch_deg, roll_deg;
extern float gx, gy, gz, ax, ay, az;
extern float gx_offset, gy_offset, gz_offset;
extern float gyro_scale;
extern float accel_scale;

extern uint8_t flag_run;

extern SEGWAY_Typedef segway;


#endif
