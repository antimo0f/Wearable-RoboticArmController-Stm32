#ifdef __cplusplus
extern "C" {
#endif

#include "joystick.h"
#include "bobot.h"
#include <stdint.h>
#include <stdio.h>


#define N_ADCPIN	4
#define INCREMENT	0.5f

#define THMAX		3000
#define THMIN		1000


uint16_t adc_buffer[N_ADCPIN];


void JOYSTICK_init(TIM_HandleTypeDef* htim4,ADC_HandleTypeDef* hadc1){

	HAL_ADC_Start_DMA(hadc1, (uint32_t*)adc_buffer, 4);
	HAL_Delay(20);
	HAL_TIM_Base_Start_IT(htim4);
}

void JOYSTICK_enable(TIM_HandleTypeDef* htim4){
	HAL_TIM_Base_Start_IT(htim4);
	HAL_Delay(20);
};

void JOYSTICK_disable(TIM_HandleTypeDef* htim4){
	HAL_TIM_Base_Stop_IT(htim4);
	HAL_Delay(20);
};

void JOYSTICK_handler(int state){

	// #define BACKUP	0
	//  #define NORMAL	1
	//
	//printf("J1_X: %4d | J1_Y: %4d | J2_Y: %4d | J2_X: %4d\r\n",
	//             adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3]);
	//printf("-----------------------------------------\r\n");

	float jvalue[4]={0,0,0,0};  //J1X J1Y,J2Y,J2X

	for(int i=0;i<4;i++){

		if(adc_buffer[i]>=THMAX){
			jvalue[i]=INCREMENT;

		}else if(adc_buffer[i]<=THMIN){
			jvalue[i]=-INCREMENT;
		}
	}
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET){
		BOBOT_effector();
	}

	if(state==0){
		BOBOT_move_relative(jvalue[0],jvalue[1],jvalue[3],jvalue[2],0);
	}else {
		BOBOT_move_relative(jvalue[0],0,0,0,0);
	}

}


/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4) {
		JOYSTICK_handler(state);
	}
}
*/
#ifdef __cplusplus
}
#endif
