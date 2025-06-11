#ifndef EPL_CLOCK_H
#define EPL_CLOCK_H

#include <stm32f0xx.h>

#define APB_FREQ 48000000
#define AHB_FREQ 48000000

void SystemClock_Config(void);

#endif // EPL_CLOCK_H
