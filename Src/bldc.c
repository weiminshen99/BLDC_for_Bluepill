
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "sysinit.h"
#include "bldc.h"


const uint8_t hall_to_pos[8] = { 0, 0, 2, 1, 4, 5, 3, 0 };

const uint8_t hall_sequence[6] = { 1, 3, 2, 6, 4, 5};

// ===========================================================================
inline void hall_to_action_test(uint8_t next_hall, int torquePWM, int *a, int *b, int *c) 
{  // next_hall is a binary string: Hc_Hb_Ha
   /*     Hx/Px(next)     Action
        =========================
        001/P0          A4 (b->c)
        010/P2          A2 (a->b)
        011/P1          A1 (c->b)
        100/P4          A0 (a->c)
        101/P5          A5 (b->a)
        110/P3          A0 (c->a)
   */
   switch(next_hall) {
      case 1: // '\001':
	*a = 0;
	*b = torquePWM;
	*c = -torquePWM;
	break;
      case 2: // '\010':
	*a = torquePWM;
	*b = -torquePWM;
	*c = 0;
	break;
      case 3: // '\011':
	*a = 0;
	*b = -torquePWM;
	*c = torquePWM;
	break;
      case 4: // '\100':
	*a = torquePWM;
	*b = 0;
	*c = -torquePWM;
	break;
      case 5: // '\101':
	*a = -torquePWM;
	*b = torquePWM;
	*c = 0;
	break;
      case 6: // '\110':
	*a = -torquePWM;
	*b = 0;
	*c = torquePWM;
      default:
	*a = torquePWM;
	*b = -torquePWM;
	*c = 0;
   }
}

// ===========================================================================
void hall_to_action(uint8_t hall, int torquePWM, int *a, int *b, int *c)
{
   /*     HcHbHa     Action
        =========================
        001          (c->a)
        010          (a->b)
        011          (c->b)
        100          (b->c)
        101          (b->a)
        110          (a->c)
   */

   int index = ( hall_to_pos[hall]+2 ) % 6;

   switch(index) {
      case 0:
	*a = 0;	*b = torquePWM;	*c = -torquePWM; // b->c
	break;
      case 1:
	*a = -torquePWM; *b = torquePWM; *c = 0; // b->a
	break;
      case 2:
	*a = -torquePWM; *b = 0; *c = torquePWM; // c->a
	break;
      case 3:
	*a = 0;	*b = -torquePWM; *c = torquePWM; // c->b
	break;
      case 4:
	*a = torquePWM;	*b = -torquePWM; *c = 0; // a->b
	break;
      case 5:
	*a = torquePWM;	*b = 0;	*c = -torquePWM; // a->c
	break;
      default:
	*a = 0; *b = torquePWM;	*c = -torquePWM;
   }
}

// ===========================================================================
void hall_to_action_2(uint8_t hall, int torquePWM, int *a, int *b, int *c)
{
   /*     HcHbHa     Action
        =========================
        001          (c->a)
        011          (c->b)
        010          (a->b)
        110          (a->c)
        100          (b->c)
        101          (b->a)
   */
   switch(hall) {
      case 4:
	*a = 0;	*b = torquePWM;	*c = -torquePWM; // b->c
	break;
      case 2:
	*a = torquePWM;	*b = -torquePWM; *c = 0; // a->b
	break;
      case 3:
	*a = 0;	*b = -torquePWM; *c = torquePWM; // c->b
	break;
      case 6:
	*a = torquePWM;	*b = 0;	*c = -torquePWM; // a->c
	break;
      case 5:
	*a = -torquePWM; *b = torquePWM; *c = 0; // b->a
	break;
      case 1:
	*a = -torquePWM; *b = 0; *c = torquePWM; // c->a
      default:
	*a = torquePWM;	*b = -torquePWM; *c = 0;
   }
}

// ================================================================
inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

// ======================================================
void Trap_BLDC_Step(uint8_t simulated_hall_pos)
{

//  int last_pos = 0;
//  int timer = 0;
//  const int max_time = PWM_FREQ / 10;
//  volatile int vel = 0;

//  uint32_t buzzerFreq = 0;
//  uint32_t buzzerPattern = 0;

  if (State.Status != READY) return;

  int ur, vr, wr;
  volatile int posr = 0;
  volatile int weakr = 0;
  volatile int pwmr = 0;

  pwmr = State.TorquePWM_desired;

  // for simulation
  if (simulated_hall_pos>=0 && simulated_hall_pos<=6) {
	posr = simulated_hall_pos;
  } else {
	posr = (hall_to_pos[State.H_POS_now] + 2) % 6; // a bit strange
  }


  // uint8_t buzz(uint16_t *notes, uint32_t len){
    // static uint32_t counter = 0;
    // static uint32_t timer = 0;
    // if(len == 0){
        // return(0);
    // }

    // struct {
        // uint16_t freq : 4;
        // uint16_t volume : 4;
        // uint16_t time : 8;
    // } note = notes[counter];

    // if(timer / 500 == note.time){
        // timer = 0;
        // counter++;
    // }

    // if(counter == len){
        // counter = 0;
    // }

    // timer++;
    // return(note.freq);
  // }


  //create square wave for buzzer
  /*buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }*/

  //update PWM channels based on position
//  blockPWM(pwmr, posr, &ur, &vr, &wr);	// old way to do things

  hall_to_action(State.H_POS_now, pwmr, &ur, &vr, &wr);

  int weakur, weakvr, weakwr;
  if (pwmr > 0) {	// forward
    blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {		// backword
    blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;

/*
  MOTOR_TIM->MOTOR_TIM_U = CLAMP(ur + PWM_RES/2, 10, PWM_RES-10);
  MOTOR_TIM->MOTOR_TIM_V = CLAMP(vr + PWM_RES/2, 10, PWM_RES-10);
  MOTOR_TIM->MOTOR_TIM_W = CLAMP(wr + PWM_RES/2, 10, PWM_RES-10);
*/

  // Make sure if Ix==0; turn off PWMx completely
  if (ur != 0) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  	MOTOR_TIM->MOTOR_TIM_U = CLAMP(ur + PWM_RES/2, 10, PWM_RES-10);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    }

  if (vr != 0) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  	MOTOR_TIM->MOTOR_TIM_V = CLAMP(vr + PWM_RES/2, 10, PWM_RES-10);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    }

 if (wr != 0) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  	MOTOR_TIM->MOTOR_TIM_W = CLAMP(wr + PWM_RES/2, 10, PWM_RES-10);
    } else {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    }

    State.Status = DONE;
}

// ===============================================================
void Motor_Timer_Start(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();

  htim1.Instance               = TIM1;
  htim1.Init.Period            = 64000000 / 2 / 16000; // = 2000 = 0.125 ms, (also can be set at TIM1->ARR)
  htim1.Init.Prescaler         = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) == HAL_ERROR) Error_Handler();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
  if (HAL_TIM_PWM_Init(&htim1)!=HAL_OK) Error_Handler();

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; // To trigger ADC1
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH; // TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

  // Enable all three channels for PWM output
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // Enable all three complemenary channels for PWM output
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // Now we init GPIO Pins for TIM1
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_TIM_ENABLE(&htim1);

  //TIM1->CCR1 = 1000;	// make sure PWM1 is active to trigger ADC1
}

