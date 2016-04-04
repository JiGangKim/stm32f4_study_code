#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "math.h"

#define MOTOR_DIRECTION_CW   1
#define MOTOR_DIRECTION_CCW -1

#define MOTOR_EN_HIGH_ACTIVE 1
#define MOTOR_EN_LOW_ACTIVE  0

#define MOTOR_ENABLE 1

typedef struct{
	TIM_HandleTypeDef* htim;
	uint32_t timer_channel;
	GPIO_TypeDef* gpio_direction_base;
	GPIO_TypeDef* gpio_enable_base;
	uint16_t gpio_direction_pin;
	uint16_t gpio_enable_pin;
	int8_t motor_direction;
	uint8_t motor_enable;
	int8_t motor_driverActiveState;
	int16_t motor_pwm_max;
    int16_t motor_pwm;
	float motor_duty;
}MOTOR_Typedef;



void motor_Init(MOTOR_Typedef* motor, 
					TIM_HandleTypeDef* htim, uint32_t timer_channel, 
					GPIO_TypeDef* gpio_direction_base, GPIO_TypeDef* gpio_enable_base,
					uint16_t gpio_direction_pin, uint16_t gpio_enable_pin,
					int8_t motor_direction, int8_t motor_driverActiveState,
					int16_t motor_pwm_max);

void motor_Enable(MOTOR_Typedef* motor);
;
void motor_SetDuty(MOTOR_Typedef* motor, float duty);


#endif

