#ifndef DRV8701
#define DRV8701

#include "stm32f3xx_hal.h"

typedef enum
{
  DRV8701_STOP,
  DRV8701_FORWARD,
  DRV8701_BACKWARD,
  DRV8701_TURN_CW,
  DRV8701_TURN_CCW,
}MotorDirection; 

void drv8701_init(void);

void drv8701_setspeed(uint32_t rightSpeed, uint32_t leftSpeed, MotorDirection direction);

#endif