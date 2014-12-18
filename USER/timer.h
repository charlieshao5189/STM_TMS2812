#ifndef __TIMER_H
#define	__TIMER_H

#include "stm32f10x.h"
#define CPU_FREQ    72000000
#define CARR_FREQ   6000				   // carrier frequency Hz 载波
#define TIMER1_ARR  6000           //=CLOCK/2*CARR_FRQC  载入值，为半周期	

typedef struct
{
    vs32 ui;  // phase 1 input
    vs32 vi;  // phase 2 input
    vs32 wi;  // phase 3 input
    vu16 uo;  // phase 1 output
    vu16 vo;  // phase 2 output
    vu16 wo;  // phase 3 output
    unsigned short c_cycle;
    unsigned short pul_cycle;
    unsigned short dead_time;
    signed short max;
    signed short min;
} PWM_LIMIT;


void Sin_Table_Init(void);
void Timer_Init(void);
void TIM_RCC_Config(void);
void TIM_GPIO_Config(void);
void TIM_Config(void);


#endif /* __TIMER_H */
