// Motor driver for TI DRV8701
// Driver uses four PWM channels driven by TIMER2 and TIMER4
// Drive the motors with 18Khz PWM to keep noise down and avoid to much switching loss

#include "drv8701.h"
#include "stm32f3xx_hal.h"

const uint32_t PERIOD = 1000;   // 18Mhz/1000 = 18Khz
const uint32_t PRESCALER = 4;   // 72/4 = 18 Mhz
 
static TIM_HandleTypeDef    TimHandle;
static TIM_HandleTypeDef    TimHandle2; 

static void Error_Handler(void)
{
  /* Turn LED3 on */
//  BSP_LED_On(LED3);
//  while(1)
//  {
//  }
}

void drv8701_init(void)
{
  TIM_OC_InitTypeDef sConfig;
  
  TimHandle.Instance = TIM2;
  
  TimHandle.Init.Prescaler = PRESCALER - 1;
  TimHandle.Init.Period = PERIOD - 1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  TimHandle2.Instance = TIM4;
  
  TimHandle2.Init.Prescaler = PRESCALER - 1;
  TimHandle2.Init.Period = PERIOD - 1;
  TimHandle2.Init.ClockDivision = 0;
  TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_PWM_Init(&TimHandle2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels #########################################*/ 
  /* Common configuration for all channels */
  sConfig.OCMode = TIM_OCMODE_PWM1;
  sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfig.OCFastMode = TIM_OCFAST_DISABLE;

  /* Set the pulse value for channel 1 */
  sConfig.Pulse = 0;  
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 2 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /* Set the pulse value for channel 4 */
  sConfig.Pulse = 0;
  if(HAL_TIM_PWM_ConfigChannel(&TimHandle2, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  
  /*##-3- Start PWM signals generation #######################################*/ 
  /* Start channel 1 */
  if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 2 */
  if(HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 3 */
  if(HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  /* Start channel 4 */
  if(HAL_TIM_PWM_Start(&TimHandle2, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }  
}

void drv8701_setspeed(uint32_t rightSpeed, uint32_t leftSpeed, MotorDirection direction)
{
//  assert(speed < PERIOD);
  
  switch(direction)
  {
  case DRV8701_STOP:
    TIM2->CCR2 = 0;
    TIM2->CCR4 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR4 = 0;
    break;
    
  case DRV8701_FORWARD:
    TIM2->CCR2 = rightSpeed;
    TIM2->CCR4 = 0;
    TIM4->CCR2 = leftSpeed;
    TIM4->CCR4 = 0;
    break;
    
  case DRV8701_BACKWARD:
    TIM2->CCR2 = 0;
    TIM2->CCR4 = rightSpeed;
    TIM4->CCR2 = 0;
    TIM4->CCR4 = leftSpeed;
    break;
    
  case DRV8701_TURN_CCW:
    TIM2->CCR2 = rightSpeed;
    TIM2->CCR4 = 0;
    TIM4->CCR2 = 0;
    TIM4->CCR4 = leftSpeed;
    break;
    
  case DRV8701_TURN_CW:
    TIM2->CCR2 = 0;
    TIM2->CCR4 = rightSpeed;
    TIM4->CCR2 = leftSpeed;
    TIM4->CCR4 = 0;
    break;
    
  default:
    break;
  }  
}
