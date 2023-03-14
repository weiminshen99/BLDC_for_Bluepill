
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "sysinit.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim3;

volatile uint32_t adc_buffer[5];

void ADC1_Init(void);
void DMA1_Init(void);
void TIM3_Init(void);

void ADC1_2_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void);

void Sensors_Test(uint8_t trigger);

