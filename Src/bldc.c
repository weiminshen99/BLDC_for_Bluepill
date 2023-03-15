
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "config.h"
#include "bldc.h"

volatile int posr = 0;
volatile int pwmr = 500;
volatile int weakr = 0;

extern volatile int speed;
extern volatile uint32_t timeout;

uint32_t buzzerFreq = 0;
uint32_t buzzerPattern = 0;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

const uint8_t hall_to_pos[8] = {
    0,
    0,
    2,
    1,
    4,
    5,
    3,
    0,
};

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

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}

uint32_t buzzerTimer        = 0;

int offsetcount = 0;
int offset_Ia   = 2000;
int offset_Ib   = 2000;
int offset_Iout  = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

int Ic = 0;
// int errorl = 0;
// int kp = 5;
// volatile int cmdl = 0;

int last_pos = 0;
int timer = 0;
const int max_time = PWM_FREQ / 10;
volatile int vel = 0;

// ======================================================
void Trap_BLDC_Step(uint8_t simulated_hall_pos)
{

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offset_Ia =   (adc_buffer[0] + offset_Ia) / 2;
    offset_Ib =   (adc_buffer[1] + offset_Ib) / 2;
    offset_Iout = (adc_buffer[2] + offset_Iout) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer[3] * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
  }

  //disable PWM when current limit is reached (current chopping)
  if(ABS((adc_buffer[2] - offset_Iout) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  int ur, vr, wr;

  //determine next position based on hall sensors
  uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
  uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
  uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

  uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;

  hallr = simulated_hall_pos; // for testing

  posr = hall_to_pos[hallr];
  posr += 2;
  posr %= 6;

  blockPhaseCurrent(posr, adc_buffer[0]-offset_Ia, adc_buffer[1]-offset_Ib, &Ic);

  //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
  //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);

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
  blockPWM(pwmr, posr, &ur, &vr, &wr);

  int weakur, weakvr, weakwr;
  if (pwmr > 0) {
    blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
  } else {
    blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
  }
  ur += weakur;
  vr += weakvr;
  wr += weakwr;

  RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
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

  TIM1->CCR1 = 1000;	// make sure PWM1 is active to trigger ADC1
}

