
#include "stm32f1xx_hal.h"
#include "math.h"
#include "defines.h"
#include "sysinit.h"
#include "sense.h"
#include "bldc.h"

//
// globle variables
//

// ===================================================================
inline void action_to_PWM(int pwm, int action, int *u, int *v, int *w)
{
      /*      Action	PWM
        `=====================
   	     	0	(b->c)
        	1	(b->a)
        	2	(c->a)
        	3	(c->b)
        	4	(a->b)
        	5	(a->c)
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

// ===============================================================================================
// WHEN I do this,
// the motor stays in a position with a big noise
// the Hall_value oscilate back_forth very fast,
// when I manually rotate the motor, it seems break everywhere, reluctent to move,
// when left in a position,  it stays there with noise.
inline void action_to_PWM_test(uint8_t action, int pwm, int *a, int *b, int *c)
{
   switch ( action ) {
      case 0:
	*a = -pwm/3; *b = pwm; *c = -pwm*2/3; break; // b->c,,a
      case 1:
	*a = -pwm*2/3; *b = pwm; *c = -pwm/3; break; // b->a,,c
      case 2:
	*a = -pwm*2/3; *b = -pwm/3; *c = pwm; break; // c->a,,b
      case 3:
	*a = -pwm/3; *b = -pwm*2/3; *c = pwm; break; // c->b,,a
      case 4:
	*a = pwm; *b = -pwm*2/3; *c = -pwm/3; break; // a->b,,c
      case 5:
	*a = pwm; *b = -pwm/3; *c = -pwm*2/3; break; // a->c,,b
      default:
	*a = 0; *b = 0; *c = 0;
   }
}

// ===========================================================================
void hall_pos_to_PWM(uint8_t hall_pos, int pwm, int *u, int *v, int *w)
{
   /*   H_cba	     halH_pos	Action	PWM
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
    int SinA = angle;
    int SinB = SinA + 120;
    int SinC = SinB + 120;

    SinA = SinA % 360;
    SinB = SinB % 360;
    SinC = SinC % 360;

    *u = pwm * sin( DEG(SinA) );
    *v = pwm * sin( DEG(SinB) );
    *w = pwm * sin( DEG(SinC) );
}

void rotation_to_PWM(int rotation, int pwm, int *u, int *v, int *w)
{
    // convert roations to x cycle + angles
    // then call continus sin waves for u, v, w
}

// ======================================================
void BLDC_Step(int x)
{
    if (State.Status != READY) return;

    int ur, vr, wr;

    if (x == -1) {  // -1 indicates to use State.H_POS_now
	if (State.ANGLE_target == State.ANGLE_now) {
	    State.PWM_now = 0; // stop
	    //hall_pos_to_PWM(State.H_POS_last, State.PWM_now, &ur, &vr, &wr); // stay
	}
	else {
	    if (State.ANGLE_now < State.ANGLE_target)
	        State.PWM_now =  State.PWM_Volume; // move forward
	    else if (State.ANGLE_now > State.ANGLE_target)
	    	State.PWM_now = -State.PWM_Volume; // move backward
	    hall_pos_to_PWM(State.H_POS_now, State.PWM_now, &ur, &vr, &wr);
	}

    } else {  // use x as the input value
	switch (State.InputType) {
	   case H_POS: // x is h_pos
		hall_pos_to_PWM(x, State.PWM_now, &ur, &vr, &wr);
		break;
	   case ANGLE: // x is angle [0,360]
		angle_to_PWM(x, State.PWM_now, &ur, &vr, &wr);
		break;
    	   case ROTATION: // x is a rotation R.A
		rotation_to_PWM(x, State.PWM_now, &ur, &vr, &wr);
		break;
	   default: break;
	}
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
    int weaku, weakv, weakw;
    volatile int weak = 0;
    volatile int pwm = 0;
    volatile int pos = 0;

    pos = State.H_POS_now;
    pwm = State.PWM_now;

    if (pwm > 0) {	// forward
       action_to_PWM(weak, (pos+5) % 6, &weaku, &weakv, &weakw);
    } else {		// backword
       action_to_PWM(-weak, (pos+1) % 6, &weaku, &weakv, &weakw);
    }
    ur += weaku;
    vr += weakv;
    wr += weakw;

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


