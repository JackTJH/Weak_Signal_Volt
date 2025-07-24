//
// Created by 23927 on 25-7-25.
//

#include "../Inc/data_process.h"


#include "data_process.h"
#include <string.h>
#include <math.h>



float remove_dc(float *data, const uint16_t length)
{
    float sum = 0.0f;
    for(uint16_t i = 0; i < length; i++)
    {
        sum += data[i];
    }
    const float avg = sum / (float)length;
    for(uint16_t i = 0; i < length; i++)
    {
        data[i] -= avg;
    }
    return avg; // 返回直流分量
}


/**
 * @brief 信号预处理函数：截取有效信号区间并去除直流分量
 * @param input_data 输入数据数组指针
 * @param input_length 输入数据长度
 * @param output_buffer 输出缓冲区（需要预分配FFT_LEN大小）
 * @param fft_len FFT长度
 * @return 返回去除的直流分量值
 */
float signal_preprocess(float *input_data, int input_length, float *output_buffer, int fft_len)
{
    // 找到全局最大值
    float global_max = input_data[0];
    for (int i = 0; i < input_length; i++) {
        if (input_data[i] > global_max) {
            global_max = input_data[i];
        }
    }

    float threshold = global_max * 0.9f;  // 设置阈值为最大值的90%

    // 找到第一个局部最大值
    int first_max_idx = 0;
    for (int i = 1; i < input_length - 1; i++) {
        if (input_data[i] > input_data[i-1] &&
            input_data[i] > input_data[i+1] &&
            input_data[i] >= threshold) {
            first_max_idx = i;
            break;
        }
    }

    // 从后往前找最后一个局部最大值
    int last_max_idx = input_length - 1;
    for (int i = input_length - 2; i > first_max_idx + 10; i--) {
        if (input_data[i] > input_data[i-1] &&
            input_data[i] > input_data[i+1] &&
            input_data[i] >= threshold) {
            last_max_idx = i;
            break;
        }
    }

    // 截取从第一个最大值到最后一个最大值的区间
    int valid_length = last_max_idx - first_max_idx + 1;

    // 将截取的数据复制到输出缓冲区
    memcpy(output_buffer, &input_data[first_max_idx], valid_length * sizeof(float));

    // 如果截取长度小于FFT_LEN，用零填充
    if (valid_length < fft_len) {
        memset(&output_buffer[valid_length], 0, (fft_len - valid_length) * sizeof(float));
    }

    // 去除直流分量并返回直流分量值
    float dc_component = remove_dc(output_buffer, valid_length);

    return dc_component;
}

/**
 * @brief 复制浮点数组数据
 * @param source 源数据数组指针
 * @param destination 目标数组指针
 * @param length 数据长度
 */
void copy_float_array(const float *source, float *destination, int length)
{
    memcpy(destination, source, length * sizeof(float));
}

/**
 * @brief 计算信号的有效值（RMS）
 * @param data 信号数据数组
 * @param length 数据长度
 * @return 返回计算出的RMS值
 */
float calculate_rms(const float *data, int length)
{
    float sum_of_squares = 0.0f;

    // 计算所有采样点的平方和
    for (int i = 0; i < length; i++) {
        sum_of_squares += data[i] * data[i];
    }

    // 计算平均值并开方得到RMS
    float mean_square = sum_of_squares / (float)length;
    return sqrtf(mean_square);
}

/**
 * @brief 计算信号的主频率分量
 * @param input_data 输入信号数据数组
 * @param fft_len FFT长度
 * @param sample_rate 采样率
 * @param fft_instance FFT实例指针
 * @param fft_output_buffer FFT输出缓冲区（需要预分配FFT_LEN大小）
 * @param fft_mag_buffer 幅度谱缓冲区（需要预分配FFT_LEN/2大小）
 * @return 返回检测到的峰值频率（Hz）
 */
float calculate_peak_frequency(float *input_data, int fft_len, int sample_rate,
                              arm_rfft_fast_instance_f32 *fft_instance,
                              float *fft_output_buffer, float *fft_mag_buffer)
{
    // 执行FFT变换
    arm_rfft_fast_f32(fft_instance, input_data, fft_output_buffer, 0);

    // 计算复数幅度谱
    arm_cmplx_mag_f32(fft_output_buffer, fft_mag_buffer, fft_len >> 1);

    // 找到幅度谱中的最大值和索引
    float max_value;
    uint32_t max_index;
    arm_max_f32(fft_mag_buffer, fft_len >> 1, &max_value, &max_index);

    // 计算对应的频率
    float peak_frequency = (float)max_index * ((float)sample_rate / fft_len);

    return peak_frequency / 2.0f;  // 返回实际频率
}