#include "beep.h"

// 蜂鸣器开启
void BEEP_ON(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

// 蜂鸣器关闭
void BEEP_OFF(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

// 短提示音 (100ms)
void BEEP_Short(void)
{
    BEEP_ON();
    HAL_Delay(100);
    BEEP_OFF();
}

// 长提示音 (500ms)
void BEEP_Long(void)
{
    BEEP_ON();
    HAL_Delay(500);
    BEEP_OFF();
}

// 成功提示音 (两短一长)
void BEEP_Success(void)
{
    BEEP_Short();
    HAL_Delay(100);
    BEEP_Short();
    HAL_Delay(100);
    BEEP_Long();
}

// 错误提示音 (三短)
void BEEP_Error(void)
{
    for(uint8_t i = 0; i < 3; i++)
    {
        BEEP_Short();
        if(i < 2) HAL_Delay(100);
    }
}

// 警告提示音 (快速连续)
void BEEP_Warning(void)
{
    for(uint8_t i = 0; i < 5; i++)
    {
        BEEP_ON();
        HAL_Delay(50);
        BEEP_OFF();
        HAL_Delay(50);
    }
}

// 启动提示音
void BEEP_StartUp(void)
{
    // 低-高-低音调效果（通过间隔模拟）
    for(uint8_t i = 0; i < 10; i++)
    {
        BEEP_ON();
        HAL_Delay(20);
        BEEP_OFF();
        HAL_Delay(80);
    }
    
    HAL_Delay(200);
    
    for(uint8_t i = 0; i < 10; i++)
    {
        BEEP_ON();
        HAL_Delay(20);
        BEEP_OFF();
        HAL_Delay(30);
    }
    
    HAL_Delay(200);
    
    for(uint8_t i = 0; i < 10; i++)
    {
        BEEP_ON();
        HAL_Delay(20);
        BEEP_OFF();
        HAL_Delay(80);
    }
}


// 自定义提示音
void BEEP_Custom(uint16_t on_time_ms, uint16_t off_time_ms, uint8_t repeat_count)
{
    for(uint8_t i = 0; i < repeat_count; i++)
    {
        BEEP_ON();
        HAL_Delay(on_time_ms);
        BEEP_OFF();
        if(i < repeat_count - 1) // 最后一次不延时
        {
            HAL_Delay(off_time_ms);
        }
    }
}

// 节拍器式提示音
void BEEP_Metronome(uint16_t bpm, uint8_t beats)
{
    uint16_t interval = 60000 / bpm; // 计算间隔时间(ms)
    
    for(uint8_t i = 0; i < beats; i++)
    {
        BEEP_ON();
        HAL_Delay(50);
        BEEP_OFF();
        
        if(i < beats - 1)
        {
            HAL_Delay(interval - 50);
        }
    }
}
