/**
//
//
//
**/


#include "pannelkey.h"
#include "amp.h"
#include "ADS1256.h"
#include "stdio.h"


uint8_t Global_Key_Val = 0;



//--------------------------------------------------------------------------------//
// 								单行按键部分（第一次测试）
//--------------------------------------------------------------------------------//


// 最后一行按键检测
void Key_Detect_Last()
{
	static uint8_t state = 0;
	switch(state) {
		case 0:
			if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)) state = 1;
			if(HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin)) state = 1;
			if(HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin)) state = 1;
			if(HAL_GPIO_ReadPin(KEY8_GPIO_Port, KEY8_Pin)) state = 1;
			break;
		case 1:
			if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin)) state = 2, Global_Key_Val = 2;
			if(HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin)) state = 2, Global_Key_Val = 4;
			if(HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin)) state = 2, Global_Key_Val = 6;
			if(HAL_GPIO_ReadPin(KEY8_GPIO_Port, KEY8_Pin)) state = 2, Global_Key_Val = 8;
			break;
		case 2:
			if(!HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) && !HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin)
				&& !HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) && !HAL_GPIO_ReadPin(KEY8_GPIO_Port, KEY8_Pin))
				state = 0;
			break;
		default: state = 0; break;
	}
}


// 最后一行按键行动
void Key_Action_Last(uint8_t key)
{
	// switch(key) {
	// 	case Key_Val_K4:
	// 		__set_FAULTMASK(1);//禁止所有的可屏蔽中断
	// 		NVIC_SystemReset();//软件复位
	// 		break;
	// 	case Key_Val_K9:
	// 		if(AMP_LPF_Mode_Sign == AMP_LPF_Mode_0Hz)			Set_AMP_LPF(AMP_LPF_Mode_35Hz);
	// 		else if(AMP_LPF_Mode_Sign == AMP_LPF_Mode_35Hz)		Set_AMP_LPF(AMP_LPF_Mode_100Hz);
	// 		else if(AMP_LPF_Mode_Sign == AMP_LPF_Mode_100Hz)	Set_AMP_LPF(AMP_LPF_Mode_0Hz);
	// 		break;
	// 	case Key_Val_K13:
	// 		if(Relay_IN_Channel_Sign == Relay_IN_Channel_IN1IN2)		Set_Relay_In(Relay_IN_Channel_IN3IN4);
	// 		else if(Relay_IN_Channel_Sign == Relay_IN_Channel_IN3IN4)	Set_Relay_In(Relay_IN_Channel_Board4mV);
	// 		else if(Relay_IN_Channel_Sign == Relay_IN_Channel_Board4mV)	Set_Relay_In(Relay_IN_Channel_Board9mV);
	// 		else if(Relay_IN_Channel_Sign == Relay_IN_Channel_Board9mV)	Set_Relay_In(Relay_IN_Channel_GND);
	// 		else if(Relay_IN_Channel_Sign == Relay_IN_Channel_GND) 		Set_Relay_In(Relay_IN_Channel_IN1IN2);
	// 		break;
	// 	case Key_Val_K14:
	// 		if(DG408_IN_Channel_Sign == DG408_IN_Channel_S1) 		Set_DG408_IN(DG408_IN_Channel_S2);
	// 		else if(DG408_IN_Channel_Sign == DG408_IN_Channel_S2)	Set_DG408_IN(DG408_IN_Channel_S4);
	// 		else if(DG408_IN_Channel_Sign == DG408_IN_Channel_S4)	Set_DG408_IN(DG408_IN_Channel_S5);
	// 		else if(DG408_IN_Channel_Sign == DG408_IN_Channel_S5)	Set_DG408_IN(DG408_IN_Channel_S6);
	// 		else if(DG408_IN_Channel_Sign == DG408_IN_Channel_S6)	Set_DG408_IN(DG408_IN_Channel_S7);
	// 		else if(DG408_IN_Channel_Sign == DG408_IN_Channel_S7)	Set_DG408_IN(DG408_IN_Channel_S1);
	// 		break;
	// 	// case Key_Val_K13:
	// 	// 	if(AMP_Mode_Sign == AMP_Mode_IN1IN2) AMP_Mode_Set(AMP_Mode_IN3IN4);
	// 	// 	else if(AMP_Mode_Sign == AMP_Mode_IN3IN4) AMP_Mode_Set(AMP_Mode_In_9mV);
	// 	// 	else if(AMP_Mode_Sign == AMP_Mode_In_9mV) AMP_Mode_Set(AMP_Mode_In_4mV);
	// 	// 	else if(AMP_Mode_Sign == AMP_Mode_In_4mV) AMP_Mode_Set(AMP_Mode_GND);
	// 	// 	else if(AMP_Mode_Sign == AMP_Mode_GND) AMP_Mode_Set(AMP_Mode_IN1IN2);
	// 	// 	break;
	// 	case Key_Val_K15:
	// 		if(AMP2_Times_Sign == AMP2_Times_X1)		Set_AMP2_Times(AMP2_Times_X10);
	// 		else if(AMP2_Times_Sign == AMP2_Times_X10)	Set_AMP2_Times(AMP2_Times_X100);
	// 		else if(AMP2_Times_Sign == AMP2_Times_X100)	Set_AMP2_Times(AMP2_Times_X1);
	// 		break;
	// 	case Key_Val_K16:
	// 		if(Adc_Sample_Rate_Sign == 5)			Set_ADC_DRATE(10);
	// 		else if(Adc_Sample_Rate_Sign == 10)		Set_ADC_DRATE(50);
	// 		else if(Adc_Sample_Rate_Sign == 50)		Set_ADC_DRATE(60);
	// 		else if(Adc_Sample_Rate_Sign == 60)		Set_ADC_DRATE(100);
	// 		else if(Adc_Sample_Rate_Sign == 100)	Set_ADC_DRATE(500);
	// 		else if(Adc_Sample_Rate_Sign == 500)	Set_ADC_DRATE(5);
	// 		break;
	// 	default: break;
	// }
}



//--------------------------------------------------------------------------------//
// 										矩阵按键部分
//--------------------------------------------------------------------------------//


#define 	PinSet		HAL_GPIO_WritePin
#define 	PinGet		HAL_GPIO_ReadPin

// 将所有行扫描引脚拉低
void Set_All_Row_Reset()
{
	PinSet(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET);
	PinSet(KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_RESET);
	PinSet(KEY5_GPIO_Port, KEY5_Pin, GPIO_PIN_RESET);
	PinSet(KEY7_GPIO_Port, KEY7_Pin, GPIO_PIN_RESET);
}

// 将单行行扫描引脚拉高
void Set_One_Row_Set(uint8_t row)
{
	switch(row) {
		case 1:
			PinSet(KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_RESET);
			PinSet(KEY5_GPIO_Port, KEY5_Pin, GPIO_PIN_RESET);
			PinSet(KEY7_GPIO_Port, KEY7_Pin, GPIO_PIN_RESET);
			PinSet(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			PinSet(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET);
			PinSet(KEY5_GPIO_Port, KEY5_Pin, GPIO_PIN_RESET);
			PinSet(KEY7_GPIO_Port, KEY7_Pin, GPIO_PIN_RESET);
			PinSet(KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_SET);
			break;
		case 3:
			PinSet(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET);
			PinSet(KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_RESET);
			PinSet(KEY7_GPIO_Port, KEY7_Pin, GPIO_PIN_RESET);
			PinSet(KEY5_GPIO_Port, KEY5_Pin, GPIO_PIN_SET);
			break;
		case 4:
			PinSet(KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_RESET);
			PinSet(KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_RESET);
			PinSet(KEY5_GPIO_Port, KEY5_Pin, GPIO_PIN_RESET);
			PinSet(KEY7_GPIO_Port, KEY7_Pin, GPIO_PIN_SET);
			break;
		default:
			Set_All_Row_Reset();
			break;
	}
}

// 读取列引脚
uint16_t Get_Column_Pin()
{
	if(PinGet(KEY2_GPIO_Port, KEY2_Pin)) return KEY2_Pin;
	if(PinGet(KEY4_GPIO_Port, KEY4_Pin)) return KEY4_Pin;
	if(PinGet(KEY6_GPIO_Port, KEY6_Pin)) return KEY6_Pin;
	if(PinGet(KEY8_GPIO_Port, KEY8_Pin)) return KEY8_Pin;
	return 0;
}

// 检测某一行是否有按键按下，返回检测结果
uint8_t Check_Row(uint8_t row)
{
	uint16_t temp = 0;
	uint8_t key = 0;
	Set_All_Row_Reset();
	switch(row) {
		case 1:
			Set_One_Row_Set(1);
			if(Get_Column_Pin()) {
				temp = Get_Column_Pin();
				switch(temp) {
					case KEY2_Pin: key = Key_Val_K1; break;
					case KEY4_Pin: key = Key_Val_K2; break;
					case KEY6_Pin: key = Key_Val_K3; break;
					case KEY8_Pin: key = Key_Val_K4; break;
					default:break;
				}
			}
			break;
		case 2:
			Set_One_Row_Set(2);
			if(Get_Column_Pin()) {
				temp = Get_Column_Pin();
				switch(temp) {
					case KEY2_Pin: key = Key_Val_K5; break;
					case KEY4_Pin: key = Key_Val_K6; break;
					case KEY6_Pin: key = Key_Val_K7; break;
					case KEY8_Pin: key = Key_Val_K8; break;
					default:break;
				}
			}
			break;
		case 3:
			Set_One_Row_Set(3);
			if(Get_Column_Pin()) {
				temp = Get_Column_Pin();
				switch(temp) {
					case KEY2_Pin: key = Key_Val_K9 ; break;
					case KEY4_Pin: key = Key_Val_K10; break;
					case KEY6_Pin: key = Key_Val_K11; break;
					case KEY8_Pin: key = Key_Val_K12; break;
					default:break;
				}
			}
			break;
		case 4:
			Set_One_Row_Set(4);
			if(Get_Column_Pin()) {
				temp = Get_Column_Pin();
				switch(temp) {
					case KEY2_Pin: key = Key_Val_K13; break;
					case KEY4_Pin: key = Key_Val_K14; break;
					case KEY6_Pin: key = Key_Val_K15; break;
					case KEY8_Pin: key = Key_Val_K16; break;
					default:break;
				}
			}
			break;
		default: break;
	}
	return key;
}


// 矩阵按键检测状态宏定义
#define Key_Detect_state_nokey				0
#define Key_Detect_state_keypress			1
#define Key_Detect_state_keyup1				2
#define Key_Detect_state_keyup2				3

// 矩阵按键检测函数
void Key_Detect()
{
	static uint8_t state = 0;
	Set_All_Row_Reset();
	switch(state) {
		case Key_Detect_state_nokey:
			Set_One_Row_Set(1); if(Get_Column_Pin()) state = Key_Detect_state_keypress;
			Set_One_Row_Set(2); if(Get_Column_Pin()) state = Key_Detect_state_keypress;
			Set_One_Row_Set(3); if(Get_Column_Pin()) state = Key_Detect_state_keypress;
			Set_One_Row_Set(4); if(Get_Column_Pin()) state = Key_Detect_state_keypress;
			break;
		case Key_Detect_state_keypress:
			if(Check_Row(1)) 		{ Global_Key_Val = Check_Row(1); state = Key_Detect_state_keyup1;}
			else if(Check_Row(2))	{ Global_Key_Val = Check_Row(2); state = Key_Detect_state_keyup1;}
			else if(Check_Row(3))	{ Global_Key_Val = Check_Row(3); state = Key_Detect_state_keyup1;}
			else if(Check_Row(4))	{ Global_Key_Val = Check_Row(4); state = Key_Detect_state_keyup1;}
			break;
		case Key_Detect_state_keyup1:
			if(!Check_Row(1) && !Check_Row(2) && !Check_Row(3) && !Check_Row(4)) state = Key_Detect_state_keyup2;
			break;
		case Key_Detect_state_keyup2:
			if(!Check_Row(1) && !Check_Row(2) && !Check_Row(3) && !Check_Row(4)) state = Key_Detect_state_nokey;
			break;
		default: state = Key_Detect_state_nokey; break;
	}
}

