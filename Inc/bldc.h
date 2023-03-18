#ifndef BLDC_H_
#define BLDC_H_

#include "defines.h"
#include "config.h"

extern volatile State_t State;

TIM_HandleTypeDef htim1;

void Motor_Timer_Start();
void Trap_BLDC_Step(uint8_t fake_hall_pos);

#endif


