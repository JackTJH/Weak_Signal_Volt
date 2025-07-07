//
// Created by 23927 on 25-7-7.
//

#ifndef INC_00_J_VOFA_UART_H
#define INC_00_J_VOFA_UART_H

#include "main.h" // 确保包含了HAL库的头文件

void Vofa_JustFloat_Send(UART_HandleTypeDef *huart, float *data, uint8_t num_channels);

#endif //INC_00_J_VOFA_UART_H
