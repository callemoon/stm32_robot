#ifndef HCSR04
#define HCSR04

#include "stm32f3xx_hal.h"  // uint_32

void hcsr04_init(uint32_t instance);
uint32_t hcsr04_getDistance(uint32_t instance);

#endif
