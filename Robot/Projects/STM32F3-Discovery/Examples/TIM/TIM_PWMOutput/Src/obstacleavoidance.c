#include "obstacleavoidance.h"
#include "stm32f3_discovery_gyroscope.h"

#include "wheelencoder.h"
#include "hcsr04.h"

#include "drv8701.h"

typedef enum
{
  STOP,
  START,
  NORMAL,
  SLOW,
  TURN_CW,
  TURN_CCW,
  BACKOFF
}runMode;

static const uint32_t LOOPTIME_MS = 3;  // loop time

static const uint32_t MINSPEED = 700;
static const uint32_t MAXSPEED = 900;
static const uint32_t FULLSPEED = 850;
static const uint32_t SLOWSPEED = 800;
static const uint32_t MINENCODERSPEED = 8000;  // Max time between encoder pulses to considered stopped

static const uint32_t TURNDISTANCE = 25; // Turn if distance below this value (cm)
static const uint32_t SLOWDISTANCE = 40; // Slow if distance below this value (cm)

void obstacleAvoidance_init(void)
{
  hcsr04_init(0);     // ultra sonic sensor 1
  hcsr04_init(1);     // ultra sonic sensor 2
  wheelencoder_init();// wheel encoder

  BSP_LED_Init(LED3);
  BSP_LED_Init(LED6);
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED10);
  BSP_LED_Init(LED9);
}

void obstacleAvoidance_run(void)
{
  uint32_t distanceLeft;
  uint32_t distanceRight;
  float gyroResponse[3];
  uint32_t leftSpeed;
  uint32_t rightSpeed;
  uint32_t wheelSpeed;

  runMode mode;

  while (1)
  {
    distanceLeft = hcsr04_getDistance(0);
    distanceRight = hcsr04_getDistance(1);
    wheelSpeed = wheelencoder_getSpeed(0);

    // Use gyro to adjust motor speed to drive straight
    BSP_GYRO_GetXYZ(gyroResponse);
    if(gyroResponse[2] > 0)
    {
      rightSpeed+=5;
    }
    else
    {
      rightSpeed-=5;
    }

    // Check that speed is within limits
    if(rightSpeed < MINSPEED)
    {
      rightSpeed = MINSPEED;
    }

    if(rightSpeed > MAXSPEED)
    {
      rightSpeed = MAXSPEED;
    }

    switch (mode)
    {
    case STOP:
      drv8701_setspeed(0, 0, DRV8701_STOP);
      break;

      // Start with full speed, during normal state speed is controlled to go straight
    case START:
      leftSpeed = FULLSPEED;
      rightSpeed = FULLSPEED;
      drv8701_setspeed(leftSpeed, rightSpeed, DRV8701_FORWARD);
      mode = NORMAL;
      break;

    case NORMAL:
      if ((distanceLeft < SLOWDISTANCE) || (distanceRight < SLOWDISTANCE))
      {
        rightSpeed = SLOWSPEED;
        leftSpeed = SLOWSPEED;

        mode = SLOW;
      }

      // If going too slow, try to back off
      if (wheelSpeed > MINENCODERSPEED)
      {
        mode = BACKOFF;
      }

      drv8701_setspeed(leftSpeed, rightSpeed, DRV8701_FORWARD);

      BSP_LED_Off(LED6);
      break;

    case SLOW:
      if ((distanceLeft > SLOWDISTANCE) && (distanceRight > SLOWDISTANCE))
      {
        mode = START;
      }

      if (distanceLeft < TURNDISTANCE) {
        mode = TURN_CW;
      }

      if (distanceRight < TURNDISTANCE) {
        mode = TURN_CCW;
      }

      // If going to slow, try to back off
      if (wheelSpeed > MINENCODERSPEED)
      {
        mode = BACKOFF;
      }

      drv8701_setspeed(leftSpeed, rightSpeed, DRV8701_FORWARD);

      BSP_LED_On(LED6);
      break;

    case TURN_CW:
      drv8701_setspeed(FULLSPEED, FULLSPEED, DRV8701_TURN_CW);
      HAL_Delay(300);
      drv8701_setspeed(0, 0, DRV8701_STOP);
      HAL_Delay(200);

      distanceLeft = hcsr04_getDistance(0);
      distanceRight = hcsr04_getDistance(1);

      // If no obstacle, go to normal
      if (distanceLeft > TURNDISTANCE && distanceRight > TURNDISTANCE)
      {
        mode = START;
      }
      break;

    case TURN_CCW:
      drv8701_setspeed(FULLSPEED, FULLSPEED, DRV8701_TURN_CCW);
      HAL_Delay(300);
      drv8701_setspeed(0, 0, DRV8701_STOP);
      HAL_Delay(200);

      // If no obstacle, go to normal
      distanceLeft = hcsr04_getDistance(0);
      distanceRight = hcsr04_getDistance(1);

      // We don't go to normal because we then risk to go to TURN_CW and we will just go and forth
      if (distanceLeft > TURNDISTANCE && distanceRight > TURNDISTANCE)
      {
        mode = START;
      }
      break;

    case BACKOFF:
      drv8701_setspeed(FULLSPEED, FULLSPEED, DRV8701_BACKWARD);
      HAL_Delay(500);
      drv8701_setspeed(FULLSPEED, FULLSPEED, DRV8701_TURN_CW);
      HAL_Delay(500);

      mode = START;
      break;
    }
  }

  HAL_Delay(LOOPTIME_MS);
}