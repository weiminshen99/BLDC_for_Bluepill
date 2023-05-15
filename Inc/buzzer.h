#ifndef BUZZER_H_
#define BUZZER_H_

#include "sysinit.h"
#include "defines.h"

void Buzzer_Timer_Init(void);
void Buzzer_Start(void);
void Buzzer_Volume_Set(uint32_t v);

#endif

