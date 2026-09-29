#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include "main.h"

void JOYSTICK_init(TIM_HandleTypeDef* htim4,ADC_HandleTypeDef* hadc1);
void JOYSTICK_handler(int state);

void JOYSTICK_disable(TIM_HandleTypeDef* htim4);
void JOYSTICK_enable(TIM_HandleTypeDef* htim4);


#endif /*JOYSTICK_H*/
