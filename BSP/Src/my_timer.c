#include "my_timer.h"

#include <string.h>

#include "ads1256.h"
#include "atk_md0350.h"
#include "pannelkey.h"
#include "amp.h"
#include "stm32f4xx_hal_tim.h"
#include "math.h"
#include "data_process.h"

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
extern uint8_t need_recalculate;
extern Detect_Mode_TypeDef Detect_DC_Or_AC; // 初始化为直流模式
extern DisplayMode_e current_display_mode;

extern Frequency_Calibration_TypeDef freq_40hz_cal_V;
extern Frequency_Calibration_TypeDef freq_80hz_cal_V;

extern Frequency_Calibration_TypeDef freq_40hz_cal_I;
extern Frequency_Calibration_TypeDef freq_80hz_cal_I;


#define FFT_LENGTH 1024 
uint16_t adc_buff[FFT_LENGTH];

static int last_key_val = -1;

// 定时器任务处理函数
void MultiTimer_TaskHandler(void)
{
    // 1ms任务处理
    if(MultiTimer_IsExpired(TIMER_1MS))
    {
        MultiTimer_ClearFlag(TIMER_1MS);
        switch (Global_Key_Val)
        {
            /******************************第一排矩阵按键******************************/
            case Key_Val_K13:
                if (last_key_val != Key_Val_K13) {
                    AMP_Parameters.dg408_in_channel = LNA_OUT;
                    lcd_printf(0, 32 * 6, Word_Size_32, BLUE, WHITE, "DETECT->Voltage");
                    last_key_val = Key_Val_K13;
                }
                break;
            case Key_Val_K14:
                if (last_key_val != Key_Val_K14) {
                    AMP_Parameters.dg408_in_channel = Ele_Input;
                    lcd_printf(0, 32 * 6, Word_Size_32, BLUE, WHITE, "DETECT->Current");
                    last_key_val = Key_Val_K14;
                }
                break;
            case Key_Val_K15:
                if (last_key_val != Key_Val_K15) {
                    last_key_val = Key_Val_K15;

                }
                break;
            case Key_Val_K16:
                if (last_key_val != Key_Val_K16) {
                    NVIC_SystemReset();
                }
                break;
            /******************************第一排矩阵按键******************************/

            /******************************第二排矩阵按键******************************/
            case Key_Val_K9:
                if (last_key_val != Key_Val_K9) {
                    Detect_DC_Or_AC = DETECT_DC; // 设置为直流检测模式
                    last_key_val = Key_Val_K9;
                }
                break;
            case Key_Val_K10:
                if (last_key_val != Key_Val_K10) {
                    Detect_DC_Or_AC = DETECT_AC; // 设置为交流检测模式
                    last_key_val = Key_Val_K10;
                }
                break;
            case Key_Val_K11:
                if (last_key_val != Key_Val_K11) {
                    current_display_mode = MODE_PC;
                    lcd_printf(0, 32 * 5, Word_Size_32, BLUE, WHITE, "MODE->PC ");
                    last_key_val = Key_Val_K11;
                }
                break;
            case Key_Val_K12:
                if (last_key_val != Key_Val_K12) {
                    current_display_mode = MODE_ARM;
                    lcd_printf(0, 32 * 5, Word_Size_32, BLUE, WHITE, "MODE->ARM");
                    last_key_val = Key_Val_K12;
                }
                break;
            /******************************第二排矩阵按键******************************/

            /******************************第三排矩阵按键******************************/
            case Key_Val_K5:
                if (last_key_val != Key_Val_K5) {
                    freq_40hz_cal_V.target_calibration_value = 11.0f;
                    freq_40hz_cal_V.calibration_done = 0;
                    freq_80hz_cal_V.target_calibration_value = 11.0f;
                    freq_80hz_cal_V.calibration_done = 0;
                    last_key_val = Key_Val_K5;
                    lcd_printf(0, 32 * 7, Word_Size_32, BLUE, WHITE, "Calibration->11uV");
                }
                break;
            case Key_Val_K6:
                if (last_key_val != Key_Val_K6) {
                    freq_40hz_cal_V.target_calibration_value = 31.0f;
                    freq_40hz_cal_V.calibration_done = 0;
                    freq_80hz_cal_V.target_calibration_value = 31.0f;
                    freq_80hz_cal_V.calibration_done = 0;
                    last_key_val = Key_Val_K6;
                    lcd_printf(0, 32 * 7, Word_Size_32, BLUE, WHITE, "Calibration->31uV");
                }
                break;
            case Key_Val_K7:
                if (last_key_val != Key_Val_K7) {
                    freq_40hz_cal_V.target_calibration_value = 51.0f;
                    freq_40hz_cal_V.calibration_done = 0;
                    freq_80hz_cal_V.target_calibration_value = 51.0f;
                    freq_80hz_cal_V.calibration_done = 0;
                    last_key_val = Key_Val_K7;
                    lcd_printf(0, 32 * 7, Word_Size_32, BLUE, WHITE, "Calibration->51uV");
                }
                break;
            case Key_Val_K8:
                if (last_key_val != Key_Val_K8) {
                    freq_40hz_cal_V.target_calibration_value = 71.0f;
                    freq_40hz_cal_V.calibration_done = 0;
                    freq_80hz_cal_V.target_calibration_value = 71.0f;
                    freq_80hz_cal_V.calibration_done = 0;
                    last_key_val = Key_Val_K8;
                    lcd_printf(0, 32 * 7, Word_Size_32, BLUE, WHITE, "Calibration->71uV");
                }
                break;
            /******************************第三排矩阵按键******************************/

            /******************************第四排矩阵按键******************************/
            case Key_Val_K1:
                    if (last_key_val != Key_Val_K1) {
                        freq_40hz_cal_I.target_calibration_value = 11.0f;
                        freq_40hz_cal_I.calibration_done = 0;
                        freq_80hz_cal_I.target_calibration_value = 11.0f;
                        freq_80hz_cal_I.calibration_done = 0;
                        last_key_val = Key_Val_K1;
                        lcd_printf(0, 32 * 8, Word_Size_32, BLUE, WHITE, "Current Cal->11uA");
                    }
                    break;
                case Key_Val_K2:
                    if (last_key_val != Key_Val_K2) {
                        freq_40hz_cal_I.target_calibration_value = 31.0f;
                        freq_40hz_cal_I.calibration_done = 0;
                        freq_80hz_cal_I.target_calibration_value = 31.0f;
                        freq_80hz_cal_I.calibration_done = 0;
                        last_key_val = Key_Val_K2;
                        lcd_printf(0, 32 * 8, Word_Size_32, BLUE, WHITE, "Current Cal->31uA");
                    }
                    break;
                case Key_Val_K3:
                    if (last_key_val != Key_Val_K3) {
                        freq_40hz_cal_I.target_calibration_value = 51.0f;
                        freq_40hz_cal_I.calibration_done = 0;
                        freq_80hz_cal_I.target_calibration_value = 51.0f;
                        freq_80hz_cal_I.calibration_done = 0;
                        last_key_val = Key_Val_K3;
                        lcd_printf(0, 32 * 8, Word_Size_32, BLUE, WHITE, "Current Cal->51uA");
                    }
                    break;
                case Key_Val_K4:
                    if (last_key_val != Key_Val_K4) {
                        freq_40hz_cal_I.target_calibration_value = 71.0f;
                        freq_40hz_cal_I.calibration_done = 0;
                        freq_80hz_cal_I.target_calibration_value = 71.0f;
                        freq_80hz_cal_I.calibration_done = 0;
                        // need_recalculate = 1;
                        lcd_printf(0, 32 * 8, Word_Size_32, BLUE, WHITE, "Current Cal->71uA");
                        last_key_val = Key_Val_K4;
                    }
                    break;
            /******************************第四排矩阵按键******************************/

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
        if(current_display_mode == MODE_ARM)
            need_recalculate = 1;
        // else
        //     need_recalculate = 0;
    }

    // 1s任务处理
    if(MultiTimer_IsExpired(TIMER_1S))
    {
        MultiTimer_ClearFlag(TIMER_1S);
        HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);  
    }
}
