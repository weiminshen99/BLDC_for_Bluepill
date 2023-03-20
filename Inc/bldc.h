#ifndef BLDC_H_
#define BLDC_H_

extern volatile State_t State;

TIM_HandleTypeDef htim1;

void Motor_Timer_Start();

void BLDC_Step(int inVal);

#endif


