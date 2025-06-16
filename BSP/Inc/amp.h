#ifndef __APM_H
#define __APM_H

#include "main.h"
#include <stdint.h>

// AMP板硬件低通滤波器 工作模式
#define AMP_LPF_Mode_0Hz    0
#define AMP_LPF_Mode_35Hz   35
#define AMP_LPF_Mode_100Hz  100

// 二级放大倍数
#define AMP2_Times_X1		1
#define AMP2_Times_X10		10
#define AMP2_Times_X100	    100

typedef enum
{
    OUT,
    LNA_OUT,
    VREF,
    VREF_700mV,
    VREF_70mV,
    VREF_9mV,
    AGND,
    Ele_Input,
}   DG408_Mode_TypeDef;

// AMP板工作模式
#define AMP_Mode_A1_GND     1
#define AMP_Mode_A1_9mV     2
#define AMP_Mode_A1_4_5mV   3
#define AMP_Mode_A1_In1In2  4
#define AMP_Mode_A1_In3In4  5
#define AMP_Mode_A2_In1In2  6
#define AMP_Mode_A2_In3In4  7
#define AMP_Mode_A2_7V      8
#define AMP_Mode_A2_700mV   9
#define AMP_Mode_A2_70mV    10
#define AMP_Mode_A2_9mV     11
#define AMP_Mode_A2_4_5mV   12
#define AMP_Mode_A2_GND     13
#define AMP_Mode_OUT_I      14

typedef struct
{
    uint8_t amp_lpf_mode;  // 硬件低通滤波器工作模式
    uint8_t amp_second_magnification; // 二级放大器放大倍数
    uint8_t amp_mode; // AMP工作模式
    uint8_t dg408_in_channel; // DG408输入通道
}AMP_Parameters_TypeDef;

extern uint8_t    AMP_LPF_Mode_Sig;
extern uint8_t    AMP2_Times_Sign;

void Set_DG408_IN(DG408_Mode_TypeDef DG408_IN_Channel);
void Set_AMP_Mode(DG408_Mode_TypeDef DG408_IN_Channel);
void Set_AMP_Second_Magnification(uint8_t times);
void Set_AMP_LPF(uint8_t AMP_LPF_Mode);
void AMP_Setup(AMP_Parameters_TypeDef *amp_param);



#endif
