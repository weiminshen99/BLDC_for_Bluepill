
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "sysinit.h"
#include "bldc.h"

const uint8_t hallValue_to_hallPos[8] = { 0, 0, 2, 1, 4, 5, 3, 0 };

// ===================================================================
inline void action_to_PWM(int pwm, int action, int *u, int *v, int *w)
{
   /*   Action	PWM
        ===============
        0      (b->c)
        1      (b->a)
        2      (c->a)
        3      (c->b)
        4      (a->b)
        5      (a->c)
   */
  switch (action) {
    case 0:
      *u = 0; *v = pwm; *w = -pwm; break; // b->c
    case 1:
      *u = -pwm; *v = pwm; *w = 0; break; // b->a
    case 2:
      *u = -pwm; *v = 0; *w = pwm; break; // c->a
    case 3:
      *u = 0; *v = -pwm; *w = pwm; break; // c->b
    case 4:
      *u = pwm; *v = -pwm; *w = 0; break; // a->b
    case 5:
      *u = pwm; *v = 0; *w = -pwm; break; // a->c
    default:
      *u = 0; *v = 0; *w = 0;
  }
}

// ===========================================================================
void hall_pos_to_PWM(uint8_t hall_pos, int pwm, int *u, int *v, int *w)
{
   /*   H_cba		H_pos	Action	PWM
        =======================================
        100    		0      	2 	(c->a)
        101    		1      	3 	(c->b)
        001    		2      	4 	(a->b)
        011    		3      	5 	(a->c)
        010    		4      	0 	(b->c)
        110    		5      	1 	(b->a)
   */
   int action = ( hall_pos + 2 ) % 6;
   action_to_PWM(pwm, action, u, v, w);
}


void angle_to_PWM(int angle, int pwm, int *u, int *v, int *w)
{
    // convert angle to continus sin waves for u, v, w
}

void rotation_to_PWM(int rotation, int pwm, int *u, int *v, int *w)
{
    // convert roations to x*cycle + angles
    // then call continus sin waves for u, v, w
}

// ======================================================
void BLDC_Step(int x)
{
    if (State.Status != READY) return;

    int ur, vr, wr;
    int h_pos;

    // update PWM channels based on input type and x
    if (x<0) { // use State.H_VAL_now
	h_pos = hallValue_to_hallPos[State.H_VAL_now];
	hall_pos_to_PWM(h_pos, (int)State.TorquePWM_desired, &ur, &vr, &wr);
    } else if (0<x && x<6 && State.InputType == H_VAL) { // x is h_pos
	hall_pos_to_PWM(x, State.TorquePWM_desired, &ur, &vr, &wr);
    } else if (State.InputType == H_POS) {
	hall_pos_to_PWM(x, State.TorquePWM_desired, &ur, &vr, &wr);
    } else if (State.InputType == ANGLE) {
	angle_to_PWM(x, State.TorquePWM_desired, &ur, &vr, &wr);
    } else if (State.InputType == ROTATION) {
	rotation_to_PWM(x, State.TorquePWM_desired, &ur, &vr, &wr);
    }


  //  int last_pos = 0;
  //  int timer = 0;
  //  const int max_time = PWM_FREQ / 10;
  //  volatile int vel = 0;

  //  uint32_t buzzerFreq = 0;
  //  uint32_t buzzerPattern = 0;

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

    //
    // implementing weakening
    //
    int weakur, weakvr, weakwr;
    volatile int weakr = 0;
    volatile int pwmr = State.TorquePWM_desired;
    volatile int posr = hallValue_to_hallPos[State.H_VAL_now];

    if (pwmr > 0) {	// forward
       action_to_PWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
    } else {		// backword
       action_to_PWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
    }
    ur += weakur;
    vr += weakvr;
    wr += weakwr;

    //
    // Executing PWM: If Ix==0; then turn off PWMx completely
    //                else, just updating PWM CCRx
    //
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


// ===============================================================================================
// WHEN I do this,
// the motor stays in a position with a big noise
// the Hall_value oscilate back_forth very fast,
// when I manually rotate the motor, it seems break everywhere, reluctent to move,
// when left in a position,  it stays there with noise.
inline void test_stay_action(uint8_t action, int pwm, int *a, int *b, int *c)
{
   switch ( action ) {
      case 0:
	*a = -pwm/2; *b = pwm; *c = -pwm/2; break; // b->c,a
      case 1:
	*a = -pwm/2; *b = pwm; *c = -pwm/2; break; // b->a,c
      case 2:
	*a = -pwm/2; *b = -pwm/2; *c = pwm; break; // c->a,b
      case 3:
	*a = -pwm/2; *b = -pwm/2; *c = pwm; break; // c->b,a
      case 4:
	*a = pwm; *b = -pwm/2; *c = -pwm/2; break; // a->b,c
      case 5:
	*a = pwm; *b = -pwm/2; *c = -pwm/2; break; // a->c,b
      default:
	*a = 0; *b = 0; *c = 0;
   }
}


