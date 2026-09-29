#ifndef BOBOT_STRUCTURE_H
#define BOBOT_STRUCTURE_H

#include <stdint.h>


typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_angle;
    float target_angle;
    float min_angle;
    float max_angle;
    float step_limit;
    int8_t inverse;
    float center_pulse;
    float min_pulse;
    float max_pulse;
} Joint_t;


typedef struct{
	Joint_t joints[5];

} Robot_t;



#endif /*BOBOT_STRUCTURE_H*/
