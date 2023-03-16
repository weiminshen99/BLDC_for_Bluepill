
#include "defines.h"
#include "config.h"

extern volatile State_t State;
extern const uint8_t hall_to_pos[8];

TIM_HandleTypeDef htim1;

void Motor_Timer_Start();
void Trap_BLDC_Step(uint8_t fake_hall_pos);

