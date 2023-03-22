/*
* This file is part of the Bare_FOC project.
*
* Copyright (C) 2021-2023 AARI Corp, Wei-Min Shen <weiminshen99@gmail.com>>
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include "stm32f1xx_hal.h"

#define MOTOR_H_STEP_SIZE 	4	// each H_STEP is 5 degree
#define MOTOR_H_SECTOR_SIZE	24	// each H_SECTOR is 30 degree (i.e., 6 H_STEPs)

typedef enum { READY, DONE } Status_t;

typedef enum { H_VAL, H_POS, ANGLE, ROTATION } Input_Type_t;

typedef struct {
  uint32_t Va;
  uint32_t Vb;
  uint32_t Iout;
  //uint32_t Vref;
  //uint32_t temp;
} adc_buf_t;

typedef struct {
  Status_t 	Status;
  Input_Type_t 	InputType;
  uint16_t 	Ia;
  int 		PWM_desired;
  int 		PWM_now;
  uint8_t 	H_POS_now;
  uint8_t 	H_POS_last;
  int		H_Sector_Counter;	// Each H_Sector contains 6 H_POS
  int		ANGLE_now;
  int		ANGLE_target;
  uint16_t 	SensorCalibCounter;
} State_t;

#define HALL_U_PIN GPIO_PIN_6
#define HALL_U_PORT GPIOB
#define HALL_V_PIN GPIO_PIN_7
#define HALL_V_PORT GPIOB
#define HALL_W_PIN GPIO_PIN_8
#define HALL_W_PORT GPIOB

#define MOTOR_TIM TIM1
#define MOTOR_TIM_U CCR1
#define MOTOR_TIM_UH_PIN GPIO_PIN_8
#define MOTOR_TIM_UH_PORT GPIOA
#define MOTOR_TIM_UL_PIN GPIO_PIN_13
#define MOTOR_TIM_UL_PORT GPIOB
#define MOTOR_TIM_V CCR2
#define MOTOR_TIM_VH_PIN GPIO_PIN_9
#define MOTOR_TIM_VH_PORT GPIOA
#define MOTOR_TIM_VL_PIN GPIO_PIN_14
#define MOTOR_TIM_VL_PORT GPIOB
#define MOTOR_TIM_W CCR3
#define MOTOR_TIM_WH_PIN GPIO_PIN_10
#define MOTOR_TIM_WH_PORT GPIOA
#define MOTOR_TIM_WL_PIN GPIO_PIN_15
#define MOTOR_TIM_WL_PORT GPIOB

// #define DC_CUR_ADC ADC2
// #define U_CUR_ADC ADC2
// #define V_CUR_ADC ADC2

#define DC_CUR_PIN GPIO_PIN_1
#define U_CUR_PIN GPIO_PIN_2
#define V_CUR_PIN GPIO_PIN_3

#define DC_CUR_PORT GPIOA
#define U_CUR_PORT GPIOA
#define V_CUR_PORT GPIOA

// #define DCLINK_ADC ADC3
// #define DCLINK_CHANNEL
#define DCLINK_PIN GPIO_PIN_2
#define DCLINK_PORT GPIOC
// #define DCLINK_PULLUP 30000
// #define DCLINK_PULLDOWN 1000

#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

#define BUZZER_PIN GPIO_PIN_0
#define BUZZER_PORT GPIOA

/*
#define SWITCH_PIN GPIO_PIN_1
#define SWITCH_PORT GPIOA

#define OFF_PIN GPIO_PIN_5
#define OFF_PORT GPIOA

#define BUTTON_PIN GPIO_PIN_1
#define BUTTON_PORT GPIOA

#define CHARGER_PIN GPIO_PIN_12
#define CHARGER_PORT GPIOA
*/

#define PWM_FREQ	16000   		// PWM frequency in Hz
#define PWM_RES 	64000000/2/PWM_FREQ 	// = 2000

#define DEAD_TIME	32         // PWM deadtime

#define DELAY_TIM_FREQUENCY_US 1000000

#define MOTOR_AMP_CONV_DC_AMP 0.02  // A per bit (12) on ADC.

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))


