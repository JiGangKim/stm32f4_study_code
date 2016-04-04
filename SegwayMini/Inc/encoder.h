#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define ENCODER_DIRECTION_CW   1
#define ENCODER_DIRECTION_CCW -1

typedef struct{
	GPIO_TypeDef* gpio_base_A;
	GPIO_TypeDef* gpio_base_B;
	uint16_t gpio_pin_A;
	uint16_t gpio_pin_B;
	uint8_t gpio_pin_A_val;
	uint8_t gpio_pin_B_val;
	int32_t encoder_tick;
	int32_t encoder_oneCycleTick;
	float encoder_angle;
	int8_t encoder_direction;
} ENCODER_TypeDef;

void encoder_Init(ENCODER_TypeDef* encoder, 
	GPIO_TypeDef* gpio_base_A, GPIO_TypeDef* gpio_base_B, 
	uint16_t gpio_pin_A, uint16_t gpio_pin_B, 
	int32_t encoder_oneCycleTick,
	int8_t encoder_direction
	);
void encoder_Update(ENCODER_TypeDef* encoder);
int encoder_GetTick(ENCODER_TypeDef* encoder);
float encoder_GetAngle(ENCODER_TypeDef* encoder);
int encoder_CalcTick(uint8_t gpio_pin_A_val_old,uint8_t gpio_pin_B_val_old,uint8_t gpio_pin_A_val_new,uint8_t gpio_pin_B_val_new);


#endif

