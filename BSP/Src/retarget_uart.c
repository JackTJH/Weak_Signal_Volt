#include "retarget_uart.h"
#include "usart.h"

/**
 * @brief 重定向 C 库 printf 函数到 UART1
 * @param fd 文件描述符
 * @param pBuffer 指向要发送数据的指针
 * @param size 要发送的数据大小
 * @return 返回成功发送的字节数
 */
int _write(int fd, char *pBuffer, int size)
{
    // 使用 HAL 库的阻塞式发送函数
    // HAL_MAX_DELAY 表示函数会一直等待直到发送完成或发生错误
    if (HAL_UART_Transmit(&huart1, (uint8_t *)pBuffer, size, HAL_MAX_DELAY) == HAL_OK)
    // if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)pBuffer, size) == HAL_OK)
    {
        return size; // 成功，返回发送的字节数
    }
    else
    {
        return -1; // 失败，返回错误
    }
}
//
//
// // 确保可以访问到在 usart.c 中定义的 huart1 实例
// extern UART_HandleTypeDef huart1;

// /**
//  * @brief 重定向 C 库 printf 函数到 UART1
//  * @param fd 文件描述符
//  * @param pBuffer 指向要发送数据的指针
//  * @param size 要发送的数据大小
//  * @return 返回成功发送的字节数
//  */
// int _write(int fd, char *pBuffer, int size)
// {
//     // 等待上一次DMA传输完成
//     // HAL_UART_GetState() 会检查UART是否处于忙碌状态
//     // 如果上一次的DMA还未结束，就在这里等待
//     while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//     {
//         // 可以添加超时处理以避免死锁
//     }
//
//     // 启动DMA传输
//     if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)pBuffer, size) == HAL_OK)
//     {
//         // 等待本次DMA传输完成，使_write表现为阻塞函数
//         while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY)
//         {
//             // 可以添加超时处理
//         }
//         return size; // 成功
//     }
//     else
//     {
//         return -1; // 失败
//     }
// }