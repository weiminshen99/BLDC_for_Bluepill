/*
* Copyright (C) 2022-2023 AARICO Corporation <weiminshen99@gmail.com>
*
* This file is part of the Bare_FOC project.
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

#include "defines.h"
#include "sysinit.h"
#include "buzzer.h"
#include "sense.h"
#include "bldc.h"

//
// globle variables
//
volatile State_t State = {0};	// shared everywhere

volatile int H_Sector = 0;

//
// main function
//

int main(void)
{
  HAL_Init();
  NVIC_Init();
  SystemClock_Config();

  LED_Init();

  Buzzer_Start();
  Sensors_Trigger_Start(3);
  Motor_Timer_Start();

  // State_Init()
  State.ANGLE_target = 360;
  State.PWM_desired = 300;		// [-1000, 1000]
  State.PWM_now = State.PWM_desired;	// this may change by BLDC_step
  State.SensorCalibCounter = 0;		// 1000
  State.Ia = 2000;			// 2000

  uint8_t hallValue_sequence[6] = {1,3,2,6,4,5};

  int main_loop_counter = 0;
  int timeout = 0;

  int simulation = 0;		// ==0 for demo of sensor-driven
  State.InputType = H_POS;	// Can be H_POS, ANGLE, or ROTATION, H_VAL


  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1); // turn off LED

  while (1)
  {
	//Buzzer_Volume_Set(State.Ia);
	//Buzzer_Volume_Set(adc_buffer.Va);
	//Buzzer_Volume_Set(adc_buffer.Vb);
	//Buzzer_Volume_Set(adc_buffer.Vc);
	Buzzer_Volume_Set(adc_buffer.Vref);

	State.ANGLE_target = adc_buffer.Vref - 2048;

	if (simulation==0) {
	    if (State.Status == READY) { // wait for ADC1 did its job
	       BLDC_Step(-1); // -1 means use State.H_VAL_now
	    }
        } else { // call BLDC_Step with the simulated input
	    HAL_Delay(10); // simulate the time for ADC1 reading
	    if (State.InputType == H_VAL) {
		BLDC_Step(hallValue_sequence[main_loop_counter%6]);
	    } else if (State.InputType == H_POS) {
		BLDC_Step(main_loop_counter%6);
	    } else if (State.InputType == ANGLE) {
		//BLDC_Step(main_loop_counter%360);
		BLDC_Step(State.Ia % 360);
	    } else {
		BLDC_Step(main_loop_counter%3600);
	    }
	    //HAL_Delay(100); // can we do it like step motor?
	}
    	main_loop_counter += 1;
	//HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
	timeout++;
  }
}

/*
// the followings are from bldc.c
extern uint8_t enable;
extern int pwm_res;

//int steer; // global variable for steering. -1000 to 1000
//int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage; // global variable for battery voltage

uint32_t inactivity_timeout_counter;

int32_t motor_test_direction = 1;

extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

int milli_vel_error_sum = 0;

void get_next_hall_readings(int index, uint8_t *hall_a, uint8_t *hall_b, uint8_t *hall_c)
{
     switch(index) {
	case 0: *hall_a=1; *hall_b=0; *hall_c=0; break;
	case 1: *hall_a=1; *hall_b=1; *hall_c=0; break;
	case 2: *hall_a=0; *hall_b=1; *hall_c=0; break;
	case 3: *hall_a=0; *hall_b=1; *hall_c=1; break;
	case 4: *hall_a=0; *hall_b=0; *hall_c=1; break;
	case 5: *hall_a=1; *hall_b=0; *hall_c=1; break;
	default: *hall_a=1; *hall_b=0; *hall_c=0;
     }
}

void hall_to_PWM(int pwm, int hall_a, int hall_b, int hall_c, int *u, int *v, int *w) 
{
  int minus_pwm = -pwm;;

//  minus_pwm = -1000; // testing

  if (hall_a==1 && hall_b==0 && hall_c==0) {		// if hall is 100
     *u = pwm;  *v = 0;     *w = minus_pwm;		// 	A -> C
  } else if (hall_a==1 && hall_b==1 && hall_c==0) { 	// if hall is 110
     *u = pwm;  *v = minus_pwm;  *w = 0;		// 	A -> B
  } else if (hall_a==0 && hall_b==1 && hall_c==0) { 	// if hall is 010
     *u = 0;  *v = minus_pwm;  *w = pwm;		// 	C -> B
  } else if (hall_a==0 && hall_b==1 && hall_c==1) {	// if hall is 011
     *u = minus_pwm;  *v = 0;  *w = pwm;		//	C -> A
  } else if (hall_a==0 && hall_b==0 && hall_c==1) {	// if hall is 001
     *u = minus_pwm;  *v = pwm;  *w = 0;		//	B -> A
  } else if (hall_a==1 && hall_b==0 && hall_c==1) {	// if hall is 101
     *u = 0;  *v = pwm;  *w = minus_pwm;		//	B -> C
  } else {						// otherwise
     *u = 0;  *v = 0;  *w = 0;				// do nothing
  }
}

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();

  MX_ADC1_Init();
  MX_ADC2_Init();

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  int ur, vr, wr;
  uint8_t hall_ur, hall_vr, hall_wr;

  enable = 1;  // enable motors, see bldc.c

*/
