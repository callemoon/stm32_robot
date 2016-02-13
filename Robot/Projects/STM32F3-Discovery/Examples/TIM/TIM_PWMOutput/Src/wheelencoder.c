// Driver for optical wheel encoder
// Make sure to use a schmitt trigger between processor and photo diode to get distinct pulses
// Uses timer 17 in capture compare mode to measure time between pulses
// Uses max filter of capture channel to avoid jitter

#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"

#include "wheelencoder.h"

static TIM_HandleTypeDef    TimHandle;
static TIM_HandleTypeDef    TimHandle2;


#define NUM_ENCODERS 2

static uint32_t encoderSpeed[NUM_ENCODERS] = {0,0};

static int on1 = 0;
static int on2 = 0;

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
  TimHandle.Instance = TIM17;
  
  TimHandle.Init.Period        = PERIOD - 1;
  TimHandle.Init.Prescaler     = PRESCALER - 1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle) != HAL_OK)
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
  if(HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  if(HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  HAL_TIM_Base_Start_IT(&TimHandle);

  TimHandle2.Instance = TIM1;
  
  TimHandle2.Init.Period        = PERIOD - 1;
  TimHandle2.Init.Prescaler     = PRESCALER - 1;
  TimHandle2.Init.ClockDivision = 0;
  TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle2) != HAL_OK)
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
  if(HAL_TIM_IC_ConfigChannel(&TimHandle2, &sICConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  if(HAL_TIM_IC_Start_IT(&TimHandle2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  HAL_TIM_Base_Start_IT(&TimHandle2);
}

uint32_t wheelencoder_getSpeed(uint8_t encoder)
{
//  assert(encoder < NUM_ENCODERS);

  return encoderSpeed[encoder];
}

void TIM1_TRG_COM_TIM17_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM1_UP_TIM16_IRQHandler(void);

void TIM1_CC_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle2, TIM_FLAG_CC2) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle2, TIM_IT_CC2) !=RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(&TimHandle2, TIM_IT_CC2);
        
        encoderSpeed[1] = HAL_TIM_ReadCapturedValue(&TimHandle2, TIM_CHANNEL_2);
        
        TIM1->CNT = 0;
        
        // Toggle led to show speed
        on2 = !on2;
        
//        if(on2)
//        {
//          BSP_LED_On(LED10);
//        }
//        else
//        {
//          BSP_LED_Off(LED10);
//        }
      }
    }
  }
}

void TIM1_UP_TIM16_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle2, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle2, TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&TimHandle2, TIM_IT_UPDATE);

      encoderSpeed[1] = PERIOD;    // If timer expires we have got no pulse during measurment period, set max time
    }
  }
}


// interrupt handler
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle, TIM_FLAG_CC1) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle, TIM_IT_CC1) !=RESET)
    {
      {
        __HAL_TIM_CLEAR_IT(&TimHandle, TIM_IT_CC1);
        
        encoderSpeed[0] = HAL_TIM_ReadCapturedValue(&TimHandle, TIM_CHANNEL_1);
        
        TIM17->CNT = 0;
        
        // Toggle led to show speed
        on1 = !on1;
        
//        if(on1)
//        {
//          BSP_LED_On(LED3);
//        }
//        else
//        {
//          BSP_LED_Off(LED3);
//        }
      }
    }
  }
  
  if(__HAL_TIM_GET_FLAG(&TimHandle, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle, TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&TimHandle, TIM_IT_UPDATE);

      encoderSpeed[0] = PERIOD;    // If timer expires we have got no pulse during measurment period, set max time
    }
  }
}
