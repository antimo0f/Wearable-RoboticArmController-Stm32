#ifndef BOBOT_H_
#define BOBOT_H_

#include "main.h"
#include <stdio.h>

#define OPENED		0
#define CLOSED		180

void BOBOT_init();
void BOBOT_inverse_cinematic(float x, float y, float z);
void BOBOT_inverse_cinematic_polar(float theta,float ro,float quote);
void BOBOT_set_angles(float a0, float a1, float a2, float a3, float a4);
void BOBOT_move_relative(float da0, float da1, float da2, float da3, float da4);
void BOBOT_effector();
void BOBOT_process();


#endif /* INTEGRATION_STRUCT_H_ */
