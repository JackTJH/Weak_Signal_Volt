//
// Created by 23927 on 25-7-25.
//

#include "../Inc/data_process.h"


#include "data_process.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>



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

    // // 去除直流分量并返回直流分量值
    // float dc_component = remove_dc(output_buffer, valid_length);
    // 计算直流分量但不修改数据
    float dc_component = 0.0f;
    for(int i = 0; i < valid_length; i++) {
        dc_component += output_buffer[i];
    }
    dc_component /= (float)valid_length;

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
 * @brief 计算信号的峰值频率
 * @param input_buffer 输入信号缓冲区
 * @param buffer_len 缓冲区长度
 * @param sample_rate 采样率
 * @param fft_inst FFT实例指针
 * @param fft_out FFT输出缓冲区
 * @param fft_magnitude 幅度缓冲区
 * @return 峰值频率值(Hz)
 */
float calculate_peak_frequency(float* input_buffer, uint16_t buffer_len, uint16_t sample_rate,
                              arm_rfft_fast_instance_f32* fft_inst, float* fft_out, float* fft_magnitude)
{
    // 执行快速实数FFT
    arm_rfft_fast_f32(fft_inst, input_buffer, fft_out, 0);

    // 计算复数幅度
    arm_cmplx_mag_f32(fft_out, fft_magnitude, buffer_len >> 1);

    // 找到最大值及其索引
    float max_value;
    uint32_t max_index;
    arm_max_f32(fft_magnitude, buffer_len >> 1, &max_value, &max_index);

    // 计算峰值频率
    float peak_frequency = (float)max_index * ((float)sample_rate / buffer_len);

    return peak_frequency / 2.0f;
}

// 初始化RMS平均计算结构体
void RMS_Average_Init(RMS_Average_TypeDef *rms_avg) {
    memset(rms_avg->buffer, 0, sizeof(rms_avg->buffer));
    rms_avg->index = 0;
    rms_avg->count = 0;
    rms_avg->averaged_value = 0.0f;
    rms_avg->is_ready = 0;
}


// 添加新的RMS值并计算平均值
float RMS_Average_Update(RMS_Average_TypeDef *rms_avg, float* input_buffer, uint32_t length) {
    float rms_value;
    arm_rms_f32(input_buffer, length, &rms_value);

    // 将新的RMS值存入缓冲区
    rms_avg->buffer[rms_avg->index] = rms_value;
    rms_avg->index = (rms_avg->index + 1) % RMS_AVERAGE_NUM;

    // 更新计数器
    if (rms_avg->count < RMS_AVERAGE_NUM) {
        rms_avg->count++;
    }

    // 当收集满RMS_AVERAGE_NUM个值时，计算平均值
    if (rms_avg->count == RMS_AVERAGE_NUM) {
        float sum = 0.0f;
        for (int i = 0; i < RMS_AVERAGE_NUM; i++) {
            sum += rms_avg->buffer[i];
        }
        rms_avg->averaged_value = sum / RMS_AVERAGE_NUM;
        rms_avg->is_ready = 1;
        return rms_avg->averaged_value;
    }

    rms_avg->is_ready = 0;
    return rms_value;  // 如果还没收集满，返回当前RMS值
}


// 清空RMS缓冲区（用于频率范围切换时）
void RMS_Average_Clear(RMS_Average_TypeDef *rms_avg) {
    memset(rms_avg->buffer, 0, sizeof(rms_avg->buffer));
    rms_avg->index = 0;
    rms_avg->count = 0;
    rms_avg->averaged_value = 0.0f;
    rms_avg->is_ready = 0;
}

// 频率校准初始化函数
void Frequency_Calibration_Init(Frequency_Calibration_TypeDef *freq_cal, float target_value) {
    freq_cal->calibration_done = 0;
    freq_cal->calibration_coefficient = target_value;  // 初始校准系数设为目标值
    freq_cal->last_freq_range = 0;
    freq_cal->target_calibration_value = target_value;
}

// 频率校准处理函数
float Process_Frequency_Calibration(Frequency_Calibration_TypeDef *freq_cal,
                                   RMS_Average_TypeDef *rms_avg,
                                   float peak_frequency,
                                   float target_freq,
                                   float rms_value,
                                   uint8_t range_id) {
    float freq_min = target_freq - 5.0f;
    float freq_max = target_freq + 5.0f;

    if (peak_frequency >= freq_min && peak_frequency <= freq_max) {
        // 检测到频率范围切换，清空RMS缓冲区重新开始平均
        if (freq_cal->last_freq_range != range_id) {
            RMS_Average_Clear(rms_avg);
            freq_cal->last_freq_range = range_id;
        }

        // 自校准逻辑 - 只在收集满RMS_AVERAGE_NUM次平均后执行一次
        if (!freq_cal->calibration_done && rms_avg->is_ready) {
            freq_cal->calibration_coefficient = freq_cal->target_calibration_value / rms_value;
            freq_cal->calibration_done = 1;  // 标记校准完成
        }

        return rms_value * freq_cal->calibration_coefficient;
    }

    return 0.0f;  // 频率不在范围内，返回0
}

// FFT处理器初始化函数
void FFT_Processor_Init(FFT_Processor_TypeDef *fft_proc, uint32_t sample_rate) {
    arm_rfft_fast_init_f32(&fft_proc->fft_instance, FFT_LEN);
    memset(fft_proc->fft_output, 0, sizeof(fft_proc->fft_output));
    memset(fft_proc->fft_mag, 0, sizeof(fft_proc->fft_mag));
    fft_proc->max_value = 0.0f;
    fft_proc->max_index = 0;
    fft_proc->peak_frequency = 0.0f;
    fft_proc->sample_rate = sample_rate;
}

// FFT频率计算处理函数
float FFT_Calculate_Peak_Frequency(FFT_Processor_TypeDef *fft_proc, float *input_buffer) {
    // 执行FFT变换
    arm_rfft_fast_f32(&fft_proc->fft_instance, input_buffer, fft_proc->fft_output, 0);

    // 计算复数幅度
    arm_cmplx_mag_f32(fft_proc->fft_output, fft_proc->fft_mag, FFT_LEN >> 1);

    // 跳过直流分量（索引0），从索引1开始找到最大幅度值及其索引
    arm_max_f32(&fft_proc->fft_mag[1], (FFT_LEN >> 1) - 1, &fft_proc->max_value, &fft_proc->max_index);

    // 调整索引值（因为跳过了索引0，所以实际索引需要+1）
    fft_proc->max_index += 1;

    // 计算峰值频率
    fft_proc->peak_frequency = (float)fft_proc->max_index * ((float)fft_proc->sample_rate / FFT_LEN);

    return fft_proc->peak_frequency;
}

// 噪声计算器初始化函数
void Noise_Calculator_Init(Noise_Calculator_TypeDef *noise_calc) {
    memset(noise_calc->noise_buffer, 0, sizeof(noise_calc->noise_buffer));
    memset(noise_calc->signal_removed_buffer, 0, sizeof(noise_calc->signal_removed_buffer));
    noise_calc->index = 0;
    noise_calc->count = 0;
    noise_calc->averaged_noise = 0.0f;
    noise_calc->is_ready = 0;
}

// // 去除有用信号后计算噪声函数
// float Calculate_Noise_After_Signal_Removal(Noise_Calculator_TypeDef *noise_calc,
//                                           float *original_signal,
//                                           float peak_frequency,
//                                           uint32_t length) {
//     // 复制原始信号
//     memcpy(noise_calc->signal_removed_buffer, original_signal, length * sizeof(float));
//
//     // 去除特定频率的有用信号（简单的高通/低通滤波或频域滤波）
//     // 这里使用简单的方法：去除直流分量和主要频率分量
//
//     // 1. 去除直流分量
//     float dc_mean;
//     arm_mean_f32(noise_calc->signal_removed_buffer, length, &dc_mean);
//     for (int i = 0; i < length; i++) {
//         noise_calc->signal_removed_buffer[i] -= dc_mean;
//     }
//
//     // 2. 如果是40Hz或80Hz，进一步处理去除这些频率分量
//     if ((peak_frequency >= 35.0f && peak_frequency <= 45.0f) ||
//         (peak_frequency >= 75.0f && peak_frequency <= 85.0f)) {
//         // 简单的移动平均滤波来去除主要频率分量
//         float filtered_signal[FFT_LEN];
//         int window_size = 5;
//
//         for (int i = 0; i < length; i++) {
//             float sum = 0.0f;
//             int count = 0;
//             for (int j = -window_size/2; j <= window_size/2; j++) {
//                 int idx = i + j;
//                 if (idx >= 0 && idx < length) {
//                     sum += noise_calc->signal_removed_buffer[idx];
//                     count++;
//                 }
//             }
//             filtered_signal[i] = sum / count;
//         }
//
//         // 从原信号中减去滤波后的信号，得到噪声
//         for (int i = 0; i < length; i++) {
//             noise_calc->signal_removed_buffer[i] -= filtered_signal[i];
//         }
//     }
//
//     // 计算剩余信号（噪声）的RMS值
//     float noise_rms;
//     arm_rms_f32(noise_calc->signal_removed_buffer, length, &noise_rms);
//
//     // 将新的噪声值存入缓冲区
//     noise_calc->noise_buffer[noise_calc->index] = noise_rms;
//     noise_calc->index = (noise_calc->index + 1) % RMS_AVERAGE_NUM;
//
//     // 更新计数器
//     if (noise_calc->count < RMS_AVERAGE_NUM) {
//         noise_calc->count++;
//     }
//
//     // 当收集满RMS_AVERAGE_NUM个值时，计算平均噪声
//     if (noise_calc->count == RMS_AVERAGE_NUM) {
//         float sum = 0.0f;
//         for (int i = 0; i < RMS_AVERAGE_NUM; i++) {
//             sum += noise_calc->noise_buffer[i];
//         }
//         noise_calc->averaged_noise = sum / RMS_AVERAGE_NUM;
//         noise_calc->is_ready = 1;
//         return noise_calc->averaged_noise;
//     }
//
//     noise_calc->is_ready = 0;
//     return noise_rms;  // 如果还没收集满，返回当前噪声值
// }

// 去除有用信号后计算噪声函数（优化版本）
float Calculate_Noise_After_Signal_Removal(Noise_Calculator_TypeDef *noise_calc,
                                          float *original_signal,
                                          float peak_frequency,
                                          uint32_t length) {
    // 复制原始信号
    memcpy(noise_calc->signal_removed_buffer, original_signal, length * sizeof(float));

    // 去除直流分量
    float dc_mean;
    arm_mean_f32(noise_calc->signal_removed_buffer, length, &dc_mean);
    for (int i = 0; i < length; i++) {
        noise_calc->signal_removed_buffer[i] -= dc_mean;
    }

    // 如果是40Hz或80Hz，使用FFT域精确去除这些频率分量
    if ((peak_frequency >= 35.0f && peak_frequency <= 45.0f) ||
        (peak_frequency >= 75.0f && peak_frequency <= 85.0f)) {

        static arm_rfft_fast_instance_f32 fft_inst;
        static uint8_t fft_init_done = 0;

        if (!fft_init_done) {
            arm_rfft_fast_init_f32(&fft_inst, FFT_LEN);
            fft_init_done = 1;
        }

        // 零填充处理
        if (length < FFT_LEN) {
            memset(&noise_calc->signal_removed_buffer[length], 0, (FFT_LEN - length) * sizeof(float));
        }

        float *fft_output = malloc(FFT_LEN * 2 * sizeof(float));
        if (fft_output == NULL) {
            goto calculate_rms;
        }

        // 执行正向FFT
        arm_rfft_fast_f32(&fft_inst, noise_calc->signal_removed_buffer, fft_output, 0);

        #define SAMPLE_RATE 7500
        float freq_resolution = (float)SAMPLE_RATE / FFT_LEN;  // 约1.83Hz/bin

        // 使用实际检测到的峰值频率进行精确抑制
        float target_freq = peak_frequency;  // 使用实际检测频率而非固定40/80Hz

        // 计算精确的频率bin
        int target_bin = (int)(target_freq / freq_resolution + 0.5f);  // 四舍五入

        // 抑制范围：只抑制目标bin及其相邻1个bin（总共3个bin）
        int bin_min = target_bin - 1;
        int bin_max = target_bin + 1;

        // 限制bin范围
        if (bin_min < 1) bin_min = 1;
        if (bin_max >= (FFT_LEN >> 1)) bin_max = (FFT_LEN >> 1) - 1;

        // 渐进式抑制而非完全置零（保留少量分量避免过度抑制）
        for (int bin = bin_min; bin <= bin_max; bin++) {
            float attenuation_factor;
            if (bin == target_bin) {
                attenuation_factor = 0.05f;  // 主频率衰减95%
            } else {
                attenuation_factor = 0.3f;   // 相邻频率衰减70%
            }

            fft_output[2 * bin] *= attenuation_factor;      // 实部
            fft_output[2 * bin + 1] *= attenuation_factor;  // 虚部
        }

        // 执行反向FFT
        arm_rfft_fast_f32(&fft_inst, fft_output, noise_calc->signal_removed_buffer, 1);

        free(fft_output);
    }

calculate_rms:
    // 计算剩余信号的RMS值
    float noise_rms;
    arm_rms_f32(noise_calc->signal_removed_buffer, length, &noise_rms);

    // 对噪声进行合理性检查和限制
    float original_rms;
    arm_rms_f32(original_signal, length, &original_rms);

    // 噪声不应该超过原信号的20%
    float max_reasonable_noise = original_rms * 0.01f;
    if (noise_rms > max_reasonable_noise) {
        noise_rms = max_reasonable_noise;
    }

    // 设置最小噪声底限（约为原信号的1%）
    float min_noise = original_rms * 0.001f;
    if (noise_rms < min_noise) {
        noise_rms = min_noise;
    }

    // 后续处理保持不变...
    noise_calc->noise_buffer[noise_calc->index] = noise_rms;
    noise_calc->index = (noise_calc->index + 1) % RMS_AVERAGE_NUM;

    if (noise_calc->count < RMS_AVERAGE_NUM) {
        noise_calc->count++;
    }

    if (noise_calc->count == RMS_AVERAGE_NUM) {
        float sum = 0.0f;
        for (int i = 0; i < RMS_AVERAGE_NUM; i++) {
            sum += noise_calc->noise_buffer[i];
        }
        noise_calc->averaged_noise = sum / RMS_AVERAGE_NUM;
        noise_calc->is_ready = 1;
        return noise_calc->averaged_noise;
    }

    noise_calc->is_ready = 0;
    return noise_rms;
}

// 清空噪声缓冲区
void Noise_Calculator_Clear(Noise_Calculator_TypeDef *noise_calc) {
    memset(noise_calc->noise_buffer, 0, sizeof(noise_calc->noise_buffer));
    memset(noise_calc->signal_removed_buffer, 0, sizeof(noise_calc->signal_removed_buffer));
    noise_calc->index = 0;
    noise_calc->count = 0;
    noise_calc->averaged_noise = 0.0f;
    noise_calc->is_ready = 0;
}

// 直流校准初始化函数
void DC_Calibration_Init(DC_Calibration_TypeDef *dc_cal, float target, float coeff) {
    dc_cal->calibration_done = 0;
    dc_cal->calibration_offset = 0.0f;
    dc_cal->target_value = target;
    dc_cal->coefficient = coeff;
}

// 直流校准处理函数
float Process_DC_Calibration(DC_Calibration_TypeDef *dc_cal, float dc_component) {
    // 自校准逻辑 - 只在开机后执行一次
    if (!dc_cal->calibration_done) {
        dc_cal->calibration_offset = dc_cal->target_value - (dc_component * dc_cal->coefficient);
        dc_cal->calibration_done = 1;  // 标记校准完成
    }

    // 应用校准偏置
    return dc_component * dc_cal->coefficient + dc_cal->calibration_offset;
}

// 直流信号噪声计算函数
float Calculate_DC_Noise(Noise_Calculator_TypeDef *noise_calc,
                         float *signal,
                         uint32_t length,
                         float dc_value) {
    // 复制原始信号
    memcpy(noise_calc->signal_removed_buffer, signal, length * sizeof(float));

    // 去除直流分量，剩余的就是噪声
    for (int i = 0; i < length; i++) {
        noise_calc->signal_removed_buffer[i] -= dc_value;
    }

    // 计算去除直流后的RMS值作为噪声
    float noise_rms;
    arm_rms_f32(noise_calc->signal_removed_buffer, length, &noise_rms);

    // 对直流噪声进行合理性检查（通常应该很小）
    float max_reasonable_noise = fabsf(dc_value) * 0.005f;  // 最大5%的噪声
    if (noise_rms > max_reasonable_noise && max_reasonable_noise > 0.1f) {
        noise_rms = max_reasonable_noise;
    }

    // 设置最小噪声底限
    float min_noise = 0.01f;  // 最小噪声底限
    if (noise_rms < min_noise) {
        noise_rms = min_noise;
    }

    // 存入噪声缓冲区进行平均
    noise_calc->noise_buffer[noise_calc->index] = noise_rms;
    noise_calc->index = (noise_calc->index + 1) % RMS_AVERAGE_NUM;

    if (noise_calc->count < RMS_AVERAGE_NUM) {
        noise_calc->count++;
    }

    if (noise_calc->count == RMS_AVERAGE_NUM) {
        float sum = 0.0f;
        for (int i = 0; i < RMS_AVERAGE_NUM; i++) {
            sum += noise_calc->noise_buffer[i];
        }
        noise_calc->averaged_noise = sum / RMS_AVERAGE_NUM;
        noise_calc->is_ready = 1;
        return noise_calc->averaged_noise;
    }

    noise_calc->is_ready = 0;
    return noise_rms;
}

extern Frequency_Calibration_TypeDef freq_40hz_cal_V_11_20;   // 11-20uV范围
extern Frequency_Calibration_TypeDef freq_40hz_cal_V_31_40;   // 31-40uV范围
extern Frequency_Calibration_TypeDef freq_40hz_cal_V_51_60;   // 51-60uV范围
extern Frequency_Calibration_TypeDef freq_40hz_cal_V_71_80;   // 71-80uV范围

extern Frequency_Calibration_TypeDef freq_80hz_cal_V_11_20;   // 11-20uV范围
extern Frequency_Calibration_TypeDef freq_80hz_cal_V_31_40;   // 31-40uV范围
extern Frequency_Calibration_TypeDef freq_80hz_cal_V_51_60;   // 51-60uV范围
extern Frequency_Calibration_TypeDef freq_80hz_cal_V_71_80;   // 71-80uV范围


