// Driver for the hcsr04 ultra sonic distance sensor
// Uses timer 8 in capture mode to measure length of pulse which equals distance to obstacle
// Seems like hcsr04 sometimes lock up for a while and does not give a falling edge, in that case the measurment will time out and we will
// measure maximum distance

#include "hcsr04.h"
#include "stm32f3xx_hal.h"

#define INSTANCES   2

static TIM_HandleTypeDef    TimHandle[INSTANCES];
static uint32_t distance[INSTANCES] = {0, 0};
static uint32_t startTime[INSTANCES] = {0, 0};
static uint8_t measurementComplete[INSTANCES] = {0, 0};

static const uint32_t PERIOD = 60000; // 60 ms max measurement time, this includes time from tigger to response from sensor
static const uint32_t PRESCALER = 72; // 1Mhz timer, 1us resolution

static const uint32_t MAXDISTANCE = 65535;  // If we get a measurement timeout we set this distance

static uint32_t timeouts[INSTANCES] = {0,0};

static void Error_Handler(void)
{
  /* Turn LED3 on */
//  BSP_LED_On(LED3);
//  while(1)
//  {
//  }
}

void hcsr04_init(uint32_t instance)
{
  TIM_IC_InitTypeDef     sICConfig;
  
  TimHandle[0].Instance = TIM8;
  TimHandle[1].Instance = TIM15;    
  
  /* Initialize TIMx peripheral as follows:
      + Period = 0xFFFF
      + Prescaler = 0
      + ClockDivision = 0
      + Counter direction = Up
  */
  TimHandle[instance].Init.Period        = PERIOD - 1; 
  TimHandle[instance].Init.Prescaler     = PRESCALER - 1;
  TimHandle[instance].Init.ClockDivision = 0;
  TimHandle[instance].Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_IC_Init(&TimHandle[instance]) != HAL_OK)
  {
   // /* Initialization Error */
    Error_Handler();
  }
  HAL_TIM_Base_Init(&TimHandle[instance]);
  
  /*##-2- Configure the Input Capture channel ################################*/ 
  /* Configure the Input Capture of channel 2 */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;  // We want interrupt on both rising and falling edge and measures the time diff between the two
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  sICConfig.ICFilter    = 15;                       // Use filter to avoid possible noise
  
  uint32_t channel;
  
  if(instance == 0)
  {
    channel = TIM_CHANNEL_1;
  }
  else
  {
    channel = TIM_CHANNEL_2;
  }
    
  if(HAL_TIM_IC_ConfigChannel(&TimHandle[instance], &sICConfig, channel) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  if(HAL_TIM_IC_Start_IT(&TimHandle[instance], channel) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  HAL_TIM_Base_Start_IT(&TimHandle[instance]);    
}

// This function does not do any measurement, only returns last measurement
// Returns distance in centimeter
uint32_t hcsr04_getDistance(uint32_t instance)
{
  return distance[instance] / 58;
}

void TIM8_CC_IRQHandler(void);
void TIM8_UP_IRQHandler(void);
void TIM1_BRK_TIM15_IRQHandler(void);

// Interrupt handler for timer 8 timer update
// Send a trig pulse to the ultrasonic sensor every time the timer is finished (every 65.5ms)
void TIM8_UP_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle[0], TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[0], TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&TimHandle[0], TIM_IT_UPDATE);
      
      // We have a timeout and measurement is not complete
      if(!measurementComplete[0])
      {
        distance[0] = MAXDISTANCE;
        timeouts[0]++;
      }
        
      volatile int i;

      // Trigger a new measurement
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
      for(i = 0; i < 100; i++);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
      
      measurementComplete[0] = 0;
    }
  }    
}


// Interrupt handler for timer 8 capture compare
void TIM8_CC_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle[0], TIM_FLAG_CC1) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[0], TIM_IT_CC1) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(&TimHandle[0], TIM_IT_CC1);

      if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))   // Rising edge
      {
        startTime[0] = HAL_TIM_ReadCapturedValue(&TimHandle[0], TIM_CHANNEL_1);
      }
      else  // falling edge, measure time diff to first pulse
      {
        distance[0] = HAL_TIM_ReadCapturedValue(&TimHandle[0], TIM_CHANNEL_1) - startTime[0];
        measurementComplete[0] = 1;
      }
    }
  }
}

// Interrupt handler for timer 1 capture compare and update
void TIM1_BRK_TIM15_IRQHandler(void)
{
  if(__HAL_TIM_GET_FLAG(&TimHandle[1], TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[1], TIM_IT_UPDATE) !=RESET)
    { 
      __HAL_TIM_CLEAR_IT(&TimHandle[1], TIM_IT_UPDATE);
      
      if(!measurementComplete[0])
      {
        distance[1] = MAXDISTANCE;
        timeouts[1]++;
      }
      
      volatile int i;

      // Trigger a new measurement
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      for(i = 0; i < 100; i++);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
      
      measurementComplete[1] = 0;
    }
  }    
  
  if(__HAL_TIM_GET_FLAG(&TimHandle[1], TIM_FLAG_CC2) != RESET)
  {
    if(__HAL_TIM_GET_ITSTATUS(&TimHandle[1], TIM_IT_CC2) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(&TimHandle[1], TIM_IT_CC2);
      
      // rising edge, store time
      if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10))
      {
        startTime[1] = HAL_TIM_ReadCapturedValue(&TimHandle[1], TIM_CHANNEL_2);
      }
      else  // falling edge, measure time diff to first pulse
      {      
        distance[1] = HAL_TIM_ReadCapturedValue(&TimHandle[1], TIM_CHANNEL_2) - startTime[1];
        measurementComplete[1] = 1;
      }
    }
  }
}

