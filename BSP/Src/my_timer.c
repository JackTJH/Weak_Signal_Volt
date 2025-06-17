#include "my_timer.h"
#include "ads1256.h"
#include "atk_md0350.h"
#include "pannelkey.h"
#include "amp.h"
#include "stm32f4xx_hal_tim.h"

static MultTimer_t timer_system; 

void MultTimer_Init(void)
{
    // 初始化各个定时器
    timer_system.timers[TIMER_1MS].period = 1;
    timer_system.timers[TIMER_1MS].counter = 0;
    timer_system.timers[TIMER_1MS].flag = 0;
    timer_system.timers[TIMER_1MS].status = TIMER_RUNNING;
    
    timer_system.timers[TIMER_10MS].period = 10;
    timer_system.timers[TIMER_10MS].counter = 0;
    timer_system.timers[TIMER_10MS].flag = 0;
    timer_system.timers[TIMER_10MS].status = TIMER_RUNNING;
    
    timer_system.timers[TIMER_200MS].period = 200;
    timer_system.timers[TIMER_200MS].counter = 0;
    timer_system.timers[TIMER_200MS].flag = 0;
    timer_system.timers[TIMER_200MS].status = TIMER_RUNNING;
    
    timer_system.timers[TIMER_1S].period = 1000;
    timer_system.timers[TIMER_1S].counter = 0;
    timer_system.timers[TIMER_1S].flag = 0;
    timer_system.timers[TIMER_1S].status = TIMER_RUNNING;

    timer_system.htim = &htim6; 
    HAL_TIM_Base_Start_IT(timer_system.htim);
}

// 定时器更新函数（在1ms中断中调用）
void MultiTimer_Update(void)
{
    // 遍历所有定时器
    for(uint8_t i = 0; i < TIMER_MAX_NUM; i++)
    {
        if(timer_system.timers[i].status == TIMER_RUNNING)
        {
            timer_system.timers[i].counter++; 
            if(timer_system.timers[i].counter >= timer_system.timers[i].period)
            {
                timer_system.timers[i].flag = 1;
                timer_system.timers[i].status = TIMER_EXPIRED;
                timer_system.timers[i].counter = 0; 
            }
        }
    }
}

// 检查定时器是否到期
uint8_t MultiTimer_IsExpired(TimerType_e timer_type)
{
    if(timer_type < TIMER_MAX_NUM)
    {
        return timer_system.timers[timer_type].flag;
    }
    return 0;
}

// 清除定时器标志
void MultiTimer_ClearFlag(TimerType_e timer_type)
{
    if(timer_type < TIMER_MAX_NUM)
    {
        timer_system.timers[timer_type].flag = 0;
        timer_system.timers[timer_type].status = TIMER_RUNNING;
    }
}

// 启动定时器
void MultiTimer_Start(TimerType_e timer_type)
{
    if(timer_type < TIMER_MAX_NUM)
    {
        timer_system.timers[timer_type].status = TIMER_RUNNING;
        timer_system.timers[timer_type].counter = 0;
        timer_system.timers[timer_type].flag = 0;
    }
}

// 停止定时器
void MultiTimer_Stop(TimerType_e timer_type)
{
    if(timer_type < TIMER_MAX_NUM)
    {
        timer_system.timers[timer_type].status = TIMER_IDLE;
        timer_system.timers[timer_type].flag = 0;
    }
}
extern AMP_Parameters_TypeDef AMP_Parameters;

// 定时器任务处理函数
void MultiTimer_TaskHandler(void)
{
    // 1ms任务处理
    if(MultiTimer_IsExpired(TIMER_1MS))
    {
        MultiTimer_ClearFlag(TIMER_1MS);
        ADS1256_Read_Data_ISR();
        switch (Global_Key_Val) 
        {
            case Key_Val_K13:
                AMP_Parameters.dg408_in_channel = OUT; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_       OUT");
            break;
            case Key_Val_K14:
                AMP_Parameters.dg408_in_channel = LNA_OUT; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_   LNA_OUT");
            break;
            case Key_Val_K15:
                AMP_Parameters.dg408_in_channel = VREF; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_      VREF");
            break;
            case Key_Val_K16:
                AMP_Parameters.dg408_in_channel = VREF_700mV; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_VREF_700mV");
            break;
            case Key_Val_K9:
                AMP_Parameters.dg408_in_channel = VREF_70mV; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_ VREF_70mV");
            break;
            case Key_Val_K10:
                AMP_Parameters.dg408_in_channel = VREF_9mV; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_  VREF_9mV");
            break;
            case Key_Val_K11:
                AMP_Parameters.dg408_in_channel = AGND; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_      AGND");
            break;
            case Key_Val_K12:
                AMP_Parameters.dg408_in_channel = Ele_Input; 
                lcd_printf(0,32,Word_Size_32,BLUE,WHITE,"AMP_Mode_ Ele_Input");
            break;
            case Key_Val_K5:
                AMP_Parameters.amp_second_magnification = AMP2_Times_X1; 
                lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"AMP_Second_Magnification_X1  ");
            break;
            case Key_Val_K6:
                AMP_Parameters.amp_second_magnification = AMP2_Times_X10; 
                lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"AMP_Second_Magnification_X10 ");
            break;
            case Key_Val_K7:
                AMP_Parameters.amp_second_magnification = AMP2_Times_X100; 
                lcd_printf(0,32*2,Word_Size_32,BLUE,WHITE,"AMP_Second_Magnification_X100");
            break;
            case Key_Val_K1:
                AMP_Parameters.amp_lpf_mode = AMP_LPF_Mode_0Hz; 
                lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"AMP_LPF_Mode_  0Hz");
            break;
            case Key_Val_K2:
                AMP_Parameters.amp_lpf_mode = AMP_LPF_Mode_35Hz; 
                lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"AMP_LPF_Mode_ 35Hz");
            break;
            case Key_Val_K3:
                AMP_Parameters.amp_lpf_mode = AMP_LPF_Mode_100Hz; 
                lcd_printf(0,32*3,Word_Size_32,BLUE,WHITE,"AMP_LPF_Mode_100Hz");
            default: break;
        }
        Set_AMP_Mode(AMP_Parameters.dg408_in_channel);  
        Set_AMP_Second_Magnification(AMP_Parameters.amp_second_magnification);
        Set_AMP_LPF(AMP_Parameters.amp_lpf_mode); 
    }
    
    // 10ms任务处理
    if(MultiTimer_IsExpired(TIMER_10MS))
    {
        MultiTimer_ClearFlag(TIMER_10MS);
        Key_Detect();
    }
    
    // 200ms任务处理
    if(MultiTimer_IsExpired(TIMER_200MS))
    {
        MultiTimer_ClearFlag(TIMER_200MS);
        lcd_printf(0,0,Word_Size_32,BLUE,WHITE,"ADS1256_Voltage:%.5lfmV",ADS1256_DATA.Voltage*1000);
    }
    
    // 1s任务处理
    if(MultiTimer_IsExpired(TIMER_1S))
    {
        MultiTimer_ClearFlag(TIMER_1S);
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);  
    }
}
