#include "my_timer.h"

static volatile uint16_t base_time_flag = 0;
static volatile uint32_t base_counter = 0;
static volatile uint8_t base_index = 0;
static const uint32_t base_time_div[(uint8_t)BASE_NUM] = {1,10,20,40,50,100,200,500,1000};


uint16_t TimeBaseGetFlag(time_base_e base){
    return (base_time_flag & (1<<(uint8_t)base));
}

void TimeBaseSetFlag(time_base_e base){
    base_time_flag |= (1u<<(uint8_t)base);
}

void TimeBaseClearFlag(time_base_e base){
    base_time_flag &= ~(1u<<(uint8_t)base);
}

void TimeBaseUpdata(void){
    base_counter++;
    for(base_index = 0; base_index < (uint8_t)BASE_NUM; base_index++)
    {
        if(base_counter % base_time_div[base_index] == 0)
        {
            TimeBaseSetFlag((time_base_e)base_index);
//            if(base_index == (uint8_t)BASE_10MS)
//            {
//                button_process();
//            }
        }
    }
}