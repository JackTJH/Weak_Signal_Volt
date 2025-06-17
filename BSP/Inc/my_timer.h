#ifndef __MY_TIMER_H
#define __MY_TIMER_H

#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

typedef enum
{
    TIMER_1MS = 0,
    TIMER_10MS,
    TIMER_200MS,
    TIMER_1S,
    TIMER_MAX_NUM
} TimerType_e;

typedef enum
{
    TIMER_IDLE = 0,
    TIMER_RUNNING,
    TIMER_EXPIRED
} TimerStatus_e;

typedef struct
{
    uint32_t period;
    uint32_t counter;
    uint8_t flag;
    TimerStatus_e status;
} Timer_t;

typedef struct
{
    Timer_t timers[TIMER_MAX_NUM];
    TIM_HandleTypeDef *htim;
} MultTimer_t;


void MultTimer_Init(void);
void MultiTimer_Update(void);
void MultiTimer_TaskHandler(void);


#endif
