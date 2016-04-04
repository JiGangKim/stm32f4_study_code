#include "segway.h"


extern float segway_roll_inst = 0.0;

void segway_Init(SEGWAY_Typedef * segway,
ENCODER_TypeDef * encoder_right,ENCODER_TypeDef * encoder_left,
MOTOR_Typedef * motor_right,MOTOR_Typedef * motor_left,
float * roll,float * pitch,float * yaw,
float controlFreq,float K_mp,float K_mi, float K_pp,float K_pi, float K_pd, float K_wp, float K_wd, float K_yp, float K_yd){
	segway->encoder_right = encoder_right;
	segway->encoder_rightAngleBuf = 0.0f;
	segway->encoder_left = encoder_left;
	segway->encoder_leftAngleBuf = 0.0f;
	segway->motor_right = motor_right;
	segway->motor_left = motor_left;
	segway->motor_rightSpeed = 0.0f;
	segway->motor_leftSpeed = 0.0f;
	segway->roll = roll;
	segway->pitch = pitch;
	segway->yaw = yaw;
	segway->controlFreq = controlFreq;
	segway->controlPeriod = 1.0f / controlFreq;
	segway->K_mp = K_mp;
	segway->K_mi = K_mi;
	segway->K_pp = K_pp;
	segway->K_pi = K_pi;
	segway->K_pd = K_pd;
	segway->K_wp = K_wp;
	segway->K_wd = K_wd;
	segway->K_yp = K_yp;
	segway->K_yd = K_yd;
	
}

void segway_SetMotorDuty(SEGWAY_Typedef * segway,float rightDuty,float leftDuty){
	motor_SetDuty(segway->motor_right,rightDuty);
	motor_SetDuty(segway->motor_left ,leftDuty );
	return;
}

void segway_SetMotorSpeed(SEGWAY_Typedef * segway,float rightSpeed,float leftSpeed){
	segway->motor_rightSpeed = rightSpeed;
	segway->motor_leftSpeed = leftSpeed;
	return;
}

void segway_ControlSegway(SEGWAY_Typedef * segway){

	float encoder_rightDeltaAngle;
	float encoder_leftDeltaAngle;
	

	//calculate motor speed

	float segway_rightSpeedByEncoderError = (segway->K_wp)*(0.0f + encoder_GetAngle(segway->encoder_right) ) - (segway->K_wd)*(0.0f + segway->motor_rightSpeed_sense);
	float segway_leftSpeedByEncoderError = (segway->K_wp)*(0.0f + encoder_GetAngle(segway->encoder_left) ) - (segway->K_wd)*(0.0f + segway->motor_leftSpeed_sense);

	if(segway_leftSpeedByEncoderError > 3.0f){
		segway_leftSpeedByEncoderError = 3.0f;
	}
	else if(segway_leftSpeedByEncoderError < -3.0f){
		segway_leftSpeedByEncoderError = -3.0f;
	}
	
	if(segway_rightSpeedByEncoderError > 3.0f){
		segway_rightSpeedByEncoderError = 3.0f;
	}
	else if(segway_rightSpeedByEncoderError < -3.0f){
		segway_rightSpeedByEncoderError = -3.0f;
	}
	
	segway->roll_error = (segway_roll_inst - *(segway->roll));
	segway->roll_error_int += segway->roll_error;
	segway->roll_dif =  (segway->roll_buf - *(segway->roll) ) * (segway->controlFreq);
	segway->roll_buf = *(segway->roll);
	
	
	segway->motor_rightSpeed = 
		- (segway->K_pp)*segway->roll_error
		- (segway->K_pi)*segway->roll_error_int * segway->controlPeriod
		+ (segway->K_pd)*(0.0f - segway->roll_dif)+segway_rightSpeedByEncoderError;
	segway->motor_leftSpeed = 
		- (segway->K_pp)*segway->roll_error
		- (segway->K_pi)*segway->roll_error_int * segway->controlPeriod
		+ (segway->K_pd)*(0.0f - segway->roll_dif)+segway_leftSpeedByEncoderError;
		
	//get motor speed from encoder data
	encoder_rightDeltaAngle = (segway->encoder_rightAngleBuf- encoder_GetAngle(segway->encoder_right) );
	segway->motor_rightSpeed_sense = (segway->motor_rightSpeed_sense)*0.9 + encoder_rightDeltaAngle*segway->controlFreq*0.1;
	segway->encoder_rightAngleBuf = encoder_GetAngle(segway->encoder_right);
	
	encoder_leftDeltaAngle = (segway->encoder_leftAngleBuf- encoder_GetAngle(segway->encoder_left) );
	segway->motor_leftSpeed_sense = (segway->motor_leftSpeed_sense)*0.9 + encoder_leftDeltaAngle*segway->controlFreq*0.1;
	segway->encoder_leftAngleBuf = encoder_GetAngle(segway->encoder_left);

	//calculate control value

	segway->motor_rightSpeed_error = segway->motor_rightSpeed - segway->motor_rightSpeed_sense;
	segway->motor_leftSpeed_error  = segway->motor_leftSpeed  - segway->motor_leftSpeed_sense ;
	segway->motor_rightSpeed_error_int += segway->motor_rightSpeed_error;
	segway->motor_leftSpeed_error_int += segway->motor_leftSpeed_error;

//	segway_SetMotorDuty(segway,segway->motor_rightSpeed, segway->motor_leftSpeed);


	segway_SetMotorDuty(segway, 
	segway->motor_right->motor_duty * 0.6f +((segway->K_mp)*(segway->motor_rightSpeed_error) + (segway->K_mi)*(segway->motor_rightSpeed_error_int)*(segway->controlPeriod))*0.4f,
	segway->motor_left->motor_duty * 0.6f +((segway->K_mp)*(segway->motor_leftSpeed_error)  + (segway->K_mi)*(segway->motor_leftSpeed_error_int) *(segway->controlPeriod))*0.4f
	);

	//set duty
}
