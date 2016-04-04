#ifndef SEGWAY_H_
#define SEGWAY_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "encoder.h"

extern float segway_roll_inst;

typedef struct{
	ENCODER_TypeDef* encoder_right;
	float encoder_rightAngleBuf;
	float encoder_leftAngleBuf;
	ENCODER_TypeDef* encoder_left;
	MOTOR_Typedef* motor_right;
	MOTOR_Typedef* motor_left;
	float motor_rightSpeed;
	float motor_rightSpeed_sense;
	float motor_rightSpeed_error;
	float motor_rightSpeed_error_int;
	float motor_leftSpeed;
	float motor_leftSpeed_sense;
	float motor_leftSpeed_error;
	float motor_leftSpeed_error_int;
	float* roll;
	float roll_error;
	float roll_error_int;
	float roll_buf;
	float roll_dif;
	float* pitch;
	float* yaw;
	float controlFreq;
	float controlPeriod;
	float K_mp, K_mi; // PI contorl constant for motor control
	float K_pp, K_pi, K_pd, K_wp, K_wd, K_yp, K_yd;
	
}SEGWAY_Typedef;

void segway_Init(SEGWAY_Typedef* segway , 
	ENCODER_TypeDef* encoder_right, ENCODER_TypeDef* encoder_left,
	MOTOR_Typedef* motor_right, MOTOR_Typedef* motor_left,
	float* roll, float* pitch, float* yaw,
	float controlFreq,
	float K_mp, float K_mi, float K_pp, float K_pi, float K_pd, float K_wp, float K_wd, float K_yp, float K_yd
);
void segway_SetMotorDuty(SEGWAY_Typedef* segway, float rightDuty, float leftDuty);
void segway_SetMotorSpeed(SEGWAY_Typedef* segway, float rightSpeed, float leftSpeed);

void segway_ControlSpeed(SEGWAY_Typedef* segway, float rightAngularSpeed, float leftAngularSpeed);
void segway_ControlSegway(SEGWAY_Typedef* segway);



#endif
