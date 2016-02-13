// Main of robot control

#include "robotcontrol.h"

#include "drv8701.h"
#include "wheelencoder.h"
#include "hcsr04.h"
#include "qre1113.h"
#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"

#include "stdio.h"

const uint32_t LOOPTIME = 3;   // 10ms loop cycle

uint32_t SLOWSPEED = 840; // Speed in slow mode
uint32_t ULTRASLOWSPEED = 750;
uint32_t FULLSPEED = 880; // Speed in fast mode
const uint32_t MINSPEED = 700;  // Min allowed speed
const uint32_t MAXSPEED = 1000; // Max allowed speed

typedef enum
{
  STOP,
  START,
  NORMAL,
  SLOW,
  TURN_CW,
  TURN_CCW,
  BACKOFF,
  SEARCH,
  BLACKLEVEL,
  WHITELEVEL,
  SEARCH_CW,
  SEARCH_CCW,
  LEFT,
  RIGHT,
  SLOWLEFT,
  SLOWRIGHT,
  ROTATE,
  FORWARD,
}runMode;

// 0 = white
// 1 = black

int directions[] = 
{
  ROTATE,   // 0000
  RIGHT,     // 0001
  SLOWRIGHT,     // 0010
  SLOWRIGHT,     // 0011
  SLOWLEFT,   // 0100
  SLOWRIGHT,     // 0101
  FORWARD,   // 0110
  SLOWRIGHT,     // 0111
  LEFT,    // 1000
  FORWARD,   // 1001
  SLOWLEFT,    // 1010
  SLOWRIGHT,     // 1011
  SLOWLEFT,    // 1100
  SLOWLEFT,    // 1101
  SLOWLEFT,    // 1110
  STOP     // 1111
}; 

void robotControl_init(void)
{
  drv8701_init();   // motor driver
  wheelencoder_init();  // wheel encoder
  qre1113_init();   // line sensor
  
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);   // push button

  /* Configure LED3 */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED6);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED10);
  BSP_LED_Init(LED9);  
}

void robotControl_run(void)
{  
  uint32_t wheelSpeed;
  uint32_t wheelSpeed2;

  
  uint32_t leftSpeed;
  uint32_t rightSpeed;
  
  uint32_t lineSensor;
  uint32_t lineSensor2;
  uint32_t lineSensor3;
  uint32_t lineSensor4;
  
  runMode mode = STOP;
  
  uint32_t blackLevel;
  uint32_t whiteLevel;
  uint32_t threashold;
  uint32_t searchTime;

  volatile double diff;
  volatile double integratedDiff;
  
  uint32_t lineValue;
  
  while(1)
  {
    // Read sensor data
/*    distanceLeft = hcsr04_getDistance(0);
    distanceRight = hcsr04_getDistance(1);*/
    wheelSpeed = wheelencoder_getSpeed(0);
    wheelSpeed2 = wheelencoder_getSpeed(1);
    lineSensor = qre1113_getValue(0) > threashold; // left
    lineSensor2 = qre1113_getValue(1) > threashold;
    lineSensor3 = qre1113_getValue(2) > threashold;  // right
    lineSensor4 = qre1113_getValue(3) > threashold;  // right
    
    lineValue = lineSensor + (lineSensor2<<1) + (lineSensor3<<2) + (lineSensor4 <<3);
    
    // Indicate if line is detected at a certain sensor
    if(lineSensor)
    {
      BSP_LED_On(LED3);
    }
    else
    {
      BSP_LED_Off(LED3);
    }
    
    if(lineSensor2)
    {
      BSP_LED_On(LED5);
    }
    else
    {
      BSP_LED_Off(LED5);
    }
    
    if(lineSensor3)
    {
      BSP_LED_On(LED9);      
    }
    else
    {
      BSP_LED_Off(LED9);      
    }

    if(lineSensor4)
    {
      BSP_LED_On(LED10);      
    }
    else
    {
      BSP_LED_Off(LED10);      
    }

    
//     printf("%u %u %u %u\n", lineSensor, lineSensor2, lineSensor3, lineSensor4);
    

    
    
//    diff = (1.0f/wheelSpeed2) - (1.0f/wheelSpeed);
//    integratedDiff += diff;
//    
//    rightSpeed = (integratedDiff * 50000) + (diff * 100000);
    
//    if(wheelSpeed < wheelSpeed2)
//    {
//      rightSpeed-=1;
//    }
//    else
//    {
//      rightSpeed+=1;      
//    }

    if(rightSpeed < MINSPEED)
    {
      rightSpeed = MINSPEED;
    }
    
    if(rightSpeed > MAXSPEED)
    {
      rightSpeed = MAXSPEED;
    }
    
    
    // Check user button
    if(BSP_PB_GetState(BUTTON_USER))
    {
      if(mode == BLACKLEVEL)
      {
        blackLevel = qre1113_getValue(0);
        
        threashold = (blackLevel + whiteLevel) / 2;
        
        mode = START;
      }

      if(mode == STOP)
      {
        whiteLevel = qre1113_getValue(0);
        
        mode = BLACKLEVEL;
      }
      
      if(mode == NORMAL)
      {
        FULLSPEED+=10;
        
        if(FULLSPEED > 1000)
        {
          mode = STOP;
          drv8701_setspeed(0, 0, DRV8701_STOP);
          FULLSPEED = 750;
        }
      }
      
      // Debounce
      HAL_Delay(500);
    }
    
    
    switch(directions[lineValue])
    {
    case FORWARD:
//          printf("forward\n");
          drv8701_setspeed(FULLSPEED, FULLSPEED, DRV8701_FORWARD);
      break;
      
    case RIGHT:
//          printf("left\n");
          drv8701_setspeed(SLOWSPEED, SLOWSPEED, DRV8701_TURN_CCW);
      break;
      
    case LEFT:
//          printf("right\n");
          drv8701_setspeed(SLOWSPEED, SLOWSPEED, DRV8701_TURN_CW);
      break;

    case SLOWRIGHT:
//          printf("left\n");
          drv8701_setspeed(FULLSPEED, SLOWSPEED, DRV8701_FORWARD);
      break;
      
    case SLOWLEFT:
//          printf("right\n");
          drv8701_setspeed(SLOWSPEED, FULLSPEED, DRV8701_FORWARD);
      break;
      
    case STOP:
//          printf("stop\n");
          drv8701_setspeed(0, 0, DRV8701_STOP);
      break;
      
    case ROTATE:
//          printf("rotate\n");
          drv8701_setspeed(SLOWSPEED, SLOWSPEED, DRV8701_TURN_CW);
      break;
      
    };

    HAL_Delay(LOOPTIME);
  }
}

