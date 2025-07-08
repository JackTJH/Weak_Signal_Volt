//
// Created by 23927 on 25-7-7.
//

#include "00_j_vofa_uart.h"
#include "usart.h"

#include <string.h> // 需要包含 string.h 以使用 memcpy

/**
 * @brief   通过UART发送符合Vofa+ JustFloat格式的数据
 * @param   huart           指向UART_HandleTypeDef结构的指针
 * @param   data            要发送的float类型数据数组的指针
 * @param   num_channels    要发送的通道数量（float数据的个数）
 */
void Vofa_JustFloat_Send(UART_HandleTypeDef *huart, float *data, uint8_t num_channels)
{
    // Vofa+ JustFloat 格式的帧尾
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

    // 计算数据总长度（所有通道的float数据 + 4字节帧尾）
    uint16_t data_len = num_channels * sizeof(float);
    uint16_t total_len = data_len + sizeof(tail);

    // 创建一个足够大的发送缓冲区
    // 注意：如果通道数很多，请考虑使用静态缓冲区或动态分配内存
    uint8_t tx_buffer[total_len];

    // 1. 将浮点数据拷贝到发送缓冲区
    memcpy(tx_buffer, (uint8_t*)data, data_len);

    // 2. 将帧尾追加到发送缓冲区
    memcpy(tx_buffer + data_len, tail, sizeof(tail));

    // HAL_UART_Transmit(huart, tx_buffer, total_len, HAL_MAX_DELAY);

    HAL_UART_Transmit_IT(huart, tx_buffer, total_len);
    while (huart1.gState != HAL_UART_STATE_READY) {}
}

// // 定义一个足够大的静态缓冲区以容纳最大可能的数据
// // 例如，如果最多发送10个通道的float数据
// #define MAX_VOFA_CHANNELS 1
// #define VOFA_TX_BUFFER_SIZE (MAX_VOFA_CHANNELS * sizeof(float) + 4) // 4是帧尾长度
//
// /**
//  * @brief   通过UART以DMA方式发送符合Vofa+ JustFloat格式的数据
//  * @param   huart           指向UART_HandleTypeDef结构的指针
//  * @param   data            要发送的float类型数据数组的指针
//  * @param   num_channels    要发送的通道数量（float数据的个数）
//  */
// void Vofa_JustFloat_Send(UART_HandleTypeDef *huart, float *data, uint8_t num_channels)
// {
//     // 使用静态缓冲区，确保在DMA传输完成前内存不会被释放
//     static uint8_t tx_buffer[VOFA_TX_BUFFER_SIZE];
//
//     // Vofa+ JustFloat 格式的帧尾
//     const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
//
//     // 检查数据是否会溢出缓冲区
//     if (num_channels > MAX_VOFA_CHANNELS)
//     {
//         // 可以加入错误处理
//         return;
//     }
//
//     // 计算数据总长度
//     uint16_t data_len = num_channels * sizeof(float);
//     uint16_t total_len = data_len + sizeof(tail);
//
//     // 1. 将浮点数据拷贝到发送缓冲区
//     memcpy(tx_buffer, (uint8_t*)data, data_len);
//
//     // 2. 将帧尾追加到发送缓冲区
//     memcpy(tx_buffer + data_len, tail, sizeof(tail));
//
//     HAL_UART_Transmit_DMA(huart, tx_buffer, total_len);
//     while (huart->gState != HAL_UART_STATE_READY) {
//         // 等待DMA传输完成
//     }
//
// }
