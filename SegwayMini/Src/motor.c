#include "motor.h"

/** @brief enable motor structure functions
  * @param motor : pointer of MOTOR_Typedef structure
  */
void motor_Init(MOTOR_Typedef* motor, 
					TIM_HandleTypeDef* htim, uint32_t timer_channel, 
					GPIO_TypeDef* gpio_direction_base, GPIO_TypeDef* gpio_enable_base,
					uint16_t gpio_direction_pin, uint16_t gpio_enable_pin,
					int8_t motor_direction, int8_t motor_driverActiveState,
					int16_t motor_pwm_max){
	motor->htim = htim;
	motor->timer_channel = timer_channel;
	motor->gpio_direction_base = gpio_direction_base;
	motor->gpio_enable_base = gpio_enable_base;
	motor->gpio_direction_pin = gpio_direction_pin;
	motor->gpio_enable_pin = gpio_enable_pin;
	motor->motor_direction = motor_direction;
	motor->motor_driverActiveState = motor_driverActiveState;
	motor->motor_pwm_max = motor_pwm_max;
}

void motor_Enable(MOTOR_Typedef* motor){
	motor->motor_enable = MOTOR_ENABLE;
	switch( motor->motor_driverActiveState ){
	case MOTOR_EN_HIGH_ACTIVE : 
		HAL_GPIO_WritePin(motor->gpio_enable_base, motor->gpio_enable_pin, GPIO_PIN_SET);
		break;
	case MOTOR_EN_LOW_ACTIVE : 
		HAL_GPIO_WritePin(motor->gpio_enable_base, motor->gpio_enable_pin, GPIO_PIN_RESET);
		break;
	}
	HAL_TIM_PWM_Start(motor->htim,motor->timer_channel);
}

/** @brief Configure motor duty 
  * @param motor : pointer of MOTOR_Typedef structure
  * @param duty : duty value (-1.0f to 1.0f), negative value means reversed rotation.
  */
void motor_SetDuty(MOTOR_Typedef* motor, float duty){
	if(duty > 1.0){
		duty = 1.0;
	}
	if(duty < -1.0){
		duty = -1.0;
	}
	
	duty *= (float)motor->motor_direction;
	if(motor->motor_enable){
		motor->motor_duty = fabsf(duty);
		motor->motor_pwm = (uint16_t)(fabsf(duty) * motor->motor_pwm_max);
		if(duty > 0.0f){
			HAL_GPIO_WritePin(motor->gpio_direction_base, motor->gpio_direction_pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(motor->gpio_direction_base, motor->gpio_direction_pin, GPIO_PIN_RESET);
		}
		__HAL_TIM_SET_COMPARE(motor->htim,motor->timer_channel,motor->motor_pwm);
	}
	else{
		return;
	}
}

