#include "encoder.h"

void encoder_Init(ENCODER_TypeDef * encoder,
GPIO_TypeDef * gpio_base_A,GPIO_TypeDef * gpio_base_B,
uint16_t gpio_pin_A,uint16_t gpio_pin_B,
int32_t encoder_oneCycleTick,
int8_t encoder_direction){
	encoder->gpio_base_A = gpio_base_A;
	encoder->gpio_base_B = gpio_base_B;
	encoder->gpio_pin_A = gpio_pin_A;
	encoder->gpio_pin_B = gpio_pin_B;
	encoder->gpio_pin_A_val = HAL_GPIO_ReadPin(gpio_base_A, gpio_pin_A);
	encoder->gpio_pin_B_val = HAL_GPIO_ReadPin(gpio_base_B, gpio_pin_B);
	encoder->encoder_oneCycleTick = encoder_oneCycleTick;
	encoder->encoder_direction = encoder_direction;
}

void encoder_Update(ENCODER_TypeDef * encoder){
	uint8_t gpio_pin_A_val_old = encoder->gpio_pin_A_val;
	uint8_t gpio_pin_B_val_old = encoder->gpio_pin_B_val;
	encoder->gpio_pin_A_val = HAL_GPIO_ReadPin(encoder->gpio_base_A, encoder->gpio_pin_A);
	encoder->gpio_pin_B_val = HAL_GPIO_ReadPin(encoder->gpio_base_B, encoder->gpio_pin_B);
	encoder->encoder_tick +=
		encoder->encoder_direction * 
		encoder_CalcTick(gpio_pin_A_val_old, gpio_pin_B_val_old, encoder->gpio_pin_A_val, encoder->gpio_pin_B_val);
	encoder->encoder_angle = 2*PI*((float)encoder->encoder_tick)/encoder->encoder_oneCycleTick;
}

int encoder_GetTick(ENCODER_TypeDef * encoder){
	return encoder->encoder_tick;
}

float encoder_GetAngle(ENCODER_TypeDef * encoder){
	return encoder->encoder_angle;
}
int encoder_CalcTick(uint8_t gpio_pin_A_val_old, uint8_t gpio_pin_B_val_old, uint8_t gpio_pin_A_val_new, uint8_t gpio_pin_B_val_new){
	int table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
	uint8_t bin = (gpio_pin_A_val_old<<3)|(gpio_pin_B_val_old<<2)|(gpio_pin_A_val_new<<1)|(gpio_pin_B_val_new<<0);
	return table[bin];
}

