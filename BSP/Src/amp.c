#include "amp.h"
#include "main.h"
#include "stm32f4xx_hal_gpio.h"


uint8_t    AMP_LPF_Mode_Sign		= 0;
uint8_t    AMP2_Times_Sign			= 0;
uint8_t    DG408_IN_Channel_Sign	= 0;

//-----------------------------------------------------------------//
//	功    能：初始化DG408引脚，选择二级放大器输入源
//	入口参数: DG408IN通道
//	出口参数: /
//	备    注: 只能通过此函数修改DG408IC参数
//
//	DG408_IN_Channel 说明
//	S1	接入IN1的电压（就是不经过50路并联累加）
//	S2	接入50路并联累加之后的电压
//	S3	接入7V基准
//	S4	接入700mV基准
//	S5	接入70mV基准
//	S6	接入9mV或4.5mV基准
//	S7	接地
//	S8	接入电流检测电路
//
//-----------------------------------------------------------------//
void Set_DG408_IN(DG408_Mode_TypeDef DG408_IN_Channel)
{
	switch(DG408_IN_Channel) {
		case OUT:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_RESET);
			DG408_IN_Channel_Sign = OUT;
			break;
		case LNA_OUT:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_RESET);
			DG408_IN_Channel_Sign = LNA_OUT;
			break;
		case VREF:
			// V1V2电压过大，不进行设置	V3电路板设置限压可以设置
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_RESET);
			DG408_IN_Channel_Sign = VREF;
			break;
		case VREF_700mV:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_RESET);
			DG408_IN_Channel_Sign = VREF_700mV;
			break;
		case VREF_70mV:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_SET);
			DG408_IN_Channel_Sign = VREF_70mV;
			break;
		case VREF_9mV:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_SET);
			DG408_IN_Channel_Sign = VREF_9mV;
			break;
		case AGND:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_SET);
			DG408_IN_Channel_Sign = AGND;
			break;
		case Ele_Input:
			HAL_GPIO_WritePin(AMP_A0_GPIO_Port, AMP_A0_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A1_GPIO_Port, AMP_A1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AMP_A2_GPIO_Port, AMP_A2_Pin, GPIO_PIN_SET);
			DG408_IN_Channel_Sign = Ele_Input;
			break;
		default: break;
	}
}

void Set_Relay_Switch(DG408_Mode_TypeDef DG408_IN_Channel)
{
	switch(DG408_IN_Channel) 
	{
		case OUT:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		case LNA_OUT:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		case VREF:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		case VREF_700mV:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		case VREF_70mV:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		case VREF_9mV:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_SET);
			break;
		case AGND:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		case Ele_Input:
			HAL_GPIO_WritePin(RE2_GPIO_Port, RE2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE3_GPIO_Port, RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RE4_GPIO_Port, RE4_Pin, GPIO_PIN_RESET);
			break;
		default: break;
	}
}

void Set_AMP_Mode(DG408_Mode_TypeDef DG408_IN_Channel)
{
	switch(DG408_IN_Channel) 
	{
		case OUT:
			Set_DG408_IN(OUT);
			Set_Relay_Switch(OUT);
			break;
		case LNA_OUT:
			Set_DG408_IN(LNA_OUT);
			Set_Relay_Switch(LNA_OUT);
			break;
		case VREF:
			Set_DG408_IN(VREF);
			Set_Relay_Switch(VREF);
			break;
		case VREF_700mV:
			Set_DG408_IN(VREF_700mV);
			Set_Relay_Switch(VREF_700mV);
			break;
		case VREF_70mV:
			Set_DG408_IN(VREF_70mV);
			Set_Relay_Switch(VREF_70mV);
			break;
		case VREF_9mV:
			Set_DG408_IN(VREF_9mV);
			Set_Relay_Switch(VREF_9mV);
			break;
		case AGND:
			Set_DG408_IN(AGND);
			Set_Relay_Switch(AGND);
			break;
		case Ele_Input:
			Set_DG408_IN(Ele_Input);
			Set_Relay_Switch(Ele_Input);
			break;
		default: break;
	}
}


//-----------------------------------------------------------------//
//	功    能：控制二级放大器放大倍数
//	入口参数: 二级放大倍数
//	出口参数: /
//	备    注: 只能通过此函数修改二级放大倍数参数
//
//	二级放大倍数 说明
//	X1		二级放大器放大倍数为1
//	X10		二级放大器放大倍数为10
//	X100	二级放大器放大倍数为100
//
//-----------------------------------------------------------------//
void Set_AMP_Second_Magnification(uint8_t times)
{
	switch (times) {
		case AMP2_Times_X1:
			HAL_GPIO_WritePin(AMP_G10_GPIO_Port, AMP_G10_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_G100_GPIO_Port, AMP_G100_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_G1_GPIO_Port, AMP_G1_Pin, GPIO_PIN_SET);
			AMP2_Times_Sign = AMP2_Times_X1;
			break;
		case AMP2_Times_X10:
			HAL_GPIO_WritePin(AMP_G1_GPIO_Port, AMP_G1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_G100_GPIO_Port, AMP_G100_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_G10_GPIO_Port, AMP_G10_Pin, GPIO_PIN_SET);
			AMP2_Times_Sign = AMP2_Times_X10;
			break;
		case AMP2_Times_X100:
			HAL_GPIO_WritePin(AMP_G1_GPIO_Port, AMP_G1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_G10_GPIO_Port, AMP_G10_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AMP_G100_GPIO_Port, AMP_G100_Pin, GPIO_PIN_SET);
			AMP2_Times_Sign = AMP2_Times_X100;
			break;
		default: break;
	}
}

//-----------------------------------------------------------------//
//	功    能：设置AMP板的硬件低通滤波器工作模式
//	入口参数: 工作模式
//	出口参数: /
//	备    注:
//
//	AMP板硬件低通滤波器 工作模式 说明
//  AMP_LPF_Mode_0Hz		无硬件低通滤波器
//  AMP_LPF_Mode_35Hz		硬件低通滤波器，通带截止为35Hz
//  AMP_LPF_Mode_100Hz		硬件低通滤波器，通带截止为100Hz
//-----------------------------------------------------------------//
void Set_AMP_LPF(uint8_t AMP_LPF_Mode)
{
	switch(AMP_LPF_Mode) {
		case AMP_LPF_Mode_0Hz:
			AMP_LPF_Mode_Sign = AMP_LPF_Mode_0Hz;
			HAL_GPIO_WritePin(ALPF_S1_GPIO_Port, ALPF_S1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ALPF_S2_GPIO_Port, ALPF_S2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ALPF_S3_GPIO_Port, ALPF_S3_Pin, GPIO_PIN_RESET);
			break;
		case AMP_LPF_Mode_35Hz:
			AMP_LPF_Mode_Sign = AMP_LPF_Mode_35Hz;
			HAL_GPIO_WritePin(ALPF_S1_GPIO_Port, ALPF_S1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ALPF_S2_GPIO_Port, ALPF_S2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ALPF_S3_GPIO_Port, ALPF_S3_Pin, GPIO_PIN_SET);
			break;
		case AMP_LPF_Mode_100Hz:
			AMP_LPF_Mode_Sign = AMP_LPF_Mode_100Hz;
			HAL_GPIO_WritePin(ALPF_S1_GPIO_Port, ALPF_S1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ALPF_S2_GPIO_Port, ALPF_S2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(ALPF_S3_GPIO_Port, ALPF_S3_Pin, GPIO_PIN_SET);
			break;
	}
}




void AMP_Setup(AMP_Parameters_TypeDef *amp_param)
{
	Set_AMP_Mode(amp_param->dg408_in_channel);  //设置DG408输入通道为S1
	Set_AMP_Second_Magnification(amp_param->amp_second_magnification);  //设置二级放大器放大倍数
	Set_AMP_LPF(amp_param->amp_lpf_mode);  //初始化滤波器（这里设置不采用二阶有源Sallen Key硬件滤波器）
}

