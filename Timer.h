#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f4xx.h" 
#define Time3PwmPeriod    200
#define Time3PwmPrescaler 42000


 void Timer5_Init(void);
 void Time3Pwm_init(int arr,int psc);
 uint64_t millis(void);

#endif
