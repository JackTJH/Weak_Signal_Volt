#ifndef __MY_TIMER_H
#define __MY_TIMER_H

#include "main.h"

typedef enum
{
    BASE_1MS = 0u,
    BASE_10MS,
    BASE_20MS,
    BASE_40MS,
    BASE_50MS,
    BASE_100MS,
    BASE_200MS,
    BASE_500MS,
    BASE_1000MS,
    BASE_NUM,
}time_base_e;

uint16_t TimeBaseGetFlag(time_base_e base);
void TimeBaseSetFlag(time_base_e base);
void TimeBaseClearFlag(time_base_e base);
void TimeBaseUpdata(void);


#endif
