#ifndef SENSE_H_
#define SENSE_H_

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "sysinit.h"

void ADC1_Init(void);
void DMA1_Init(void);
void TIM3_Init(void);
void HALL_Init(void);

void Sensors_Trigger_Start(uint8_t trigger);

void DMA1_Channel1_IRQHandler(void);

void ADC1_2_IRQHandler(void);

void Emergency_Shut_Down(void);
int  HALL_Sense();
void Process_Raw_Sensor_Data(void);

#endif



