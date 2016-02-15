// Driver for optical wheel encoder
// Make sure to use a schmitt trigger between processor and photo diode to get distinct pulses
// Uses timer 17 and timer 1 in capture compare mode to measure time between pulses
// Uses max filter of capture channel to avoid jitter

#include "stm32f3xx_hal.h"

#include "wheelencoder.h"


#define NUM_ENCODERS 2

static TIM_HandleTypeDef    TimHandle[NUM_ENCODERS];
static uint32_t encoderSpeed[NUM_ENCODERS] = {0,0};

static const uint32_t PERIOD = 10000;   // 100us*10000 = 1000ms max measurement time
static const uint32_t PRESCALER = 7200; // 100us steps

static void Error_Handler(void)
{
  /* Turn LED3 on */
//  BSP_LED_On(LED3);
//  while(1)
//  {
//  }
}

void wheelencoder_init(void)
{
  TIM_IC_InitTypeDef     sICConfig;
  TimHandle[0].Instance = TIM17;
  
  TimHandle[0].Init.Period        = PERIOD - 1;
  TimHandle[0].Init.Prescaler     = PRESCALER - 1;
  TimHandle[0].Init.ClockDivision = 0;
  TimHandle[0].Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle[0]) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }  

   /*##-2- Configure the Input Capture channel ################################*/ 
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 15;                   // Use max filter to avoid jitter
  if(HAL_TIM_IC_ConfigChannel(&TimHandle[0], &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  if(HAL_TIM_IC_Start_IT(&TimHandle[0], TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  if(HAL_TIM_Base_Start_IT(&TimHandle[0]) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  TimHandle[1].Instance = TIM1;
  
  TimHandle[1].Init.Period        = PERIOD - 1;
  TimHandle[1].Init.Prescaler     = PRESCALER - 1;
  TimHandle[1].Init.ClockDivision = 0;
  TimHandle[1].Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle[1]) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }  

   /*##-2- Configure the Input Capture channel ################################*/ 
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 15;                   // Use max filter to avoid jitter
  if(HAL_TIM_IC_ConfigChannel(&TimHandle[1], &sICConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  if(HAL_TIM_IC_Start_IT(&TimHandle[1], TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  if(HAL_TIM_Base_Start_IT(&TimHandle[1]) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

uint32_t wheelencoder_getSpeed(uint8_t encoder)
{
//  assert(encoder < NUM_ENCODERS);

  return encoderSpeed[encoder];
}

void TIM1_TRG_COM_TIM17_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);

// Timer 1 capture compare interrupt
void TIM1_CC_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle[1], TIM_FLAG_CC2) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[1], TIM_IT_CC2) !=RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(&TimHandle[1], TIM_IT_CC2);
        
        encoderSpeed[1] = HAL_TIM_ReadCapturedValue(&TimHandle[1], TIM_CHANNEL_2);
        
        TIM1->CNT = 0;
      }
    }
  }
}

// Timer 1 Update interrupt
void TIM1_UP_TIM16_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle[1], TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[1], TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&TimHandle[1], TIM_IT_UPDATE);

      encoderSpeed[1] = PERIOD;    // If timer expires we have got no pulse during measurment period, set max time
    }
  }
}


// Timer 17 update and capture compare interrupt
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle[0], TIM_FLAG_CC1) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[0], TIM_IT_CC1) !=RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(&TimHandle[0], TIM_IT_CC1);
        
        encoderSpeed[0] = HAL_TIM_ReadCapturedValue(&TimHandle[0], TIM_CHANNEL_1);
        
        TIM17->CNT = 0;
      }
    }
  }
  
  if(__HAL_TIM_GET_FLAG(&TimHandle[0], TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[0], TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&TimHandle[0], TIM_IT_UPDATE);

      encoderSpeed[0] = PERIOD;    // If timer expires we have got no pulse during measurment period, set max time
    }
  }
}
