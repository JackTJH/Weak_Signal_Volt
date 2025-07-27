//
// Created by 23927 on 25-7-25.
//

#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include "main.h"
#include "arm_math.h"
#include <stdbool.h>

#define FFT_LEN		4096
#define SAMPLE_RATE  15000
#define RMS_AVERAGE_NUM   10
#define RMS_SELF_CAL      11.0f
#define DC_I_COEFFICIENT  0.344f

// RMS平均计算结构体定义
typedef struct {
    float buffer[RMS_AVERAGE_NUM];    // RMS值缓冲区
    uint8_t index;                    // 当前索引
    uint8_t count;                    // 已存储的RMS值数量
    float averaged_value;             // 平均后的RMS值
    uint8_t is_ready;                 // 平均值是否准备好的标志
} RMS_Average_TypeDef;

// 检测模式枚举定义
typedef enum {
    DETECT_DC = 0,     // 直流检测模式
    DETECT_AC = 1      // 交流检测模式
} Detect_Mode_TypeDef;

typedef enum {
    MODE_NONE = 0,
    MODE_PC,
    MODE_ARM
} DisplayMode_e;





// 频率校准结构体定义
typedef struct {
    uint8_t calibration_done;           // 校准完成标志
    float calibration_coefficient;      // 校准系数
    uint8_t last_freq_range;           // 上次频率范围标识
    float target_calibration_value;    // 目标校准值
} Frequency_Calibration_TypeDef;

// FFT处理结构体定义
typedef struct {
    float fft_output[FFT_LEN];              // FFT输出缓冲区
    float fft_mag[FFT_LEN >> 1];            // FFT幅度谱缓冲区
    arm_rfft_fast_instance_f32 fft_instance; // FFT实例
    float max_value;                        // 最大幅度值
    uint32_t max_index;                     // 最大值索引
    float peak_frequency;                   // 峰值频率
    uint32_t sample_rate;                   // 采样率
} FFT_Processor_TypeDef;

// 噪声计算结构体定义
typedef struct {
    float noise_buffer[RMS_AVERAGE_NUM];    // 噪声值缓冲区
    uint8_t index;                          // 当前索引
    uint8_t count;                          // 已存储的噪声值数量
    float averaged_noise;                   // 平均噪声值
    uint8_t is_ready;                       // 噪声计算是否准备好的标志
    float signal_removed_buffer[FFT_LEN];   // 去除信号后的缓冲区
} Noise_Calculator_TypeDef;

// 直流校准结构体定义
typedef struct {
    uint8_t calibration_done;           // 校准完成标志
    float calibration_offset;           // 校准偏置值
    float target_value;                 // 目标校准值
    float coefficient;                  // 校准系数（用于电流通道）
} DC_Calibration_TypeDef;


float signal_preprocess(float *input_data, int input_length, float *output_buffer, int fft_len);
void copy_float_array(const float *source, float *destination, int length);
float calculate_rms(const float *data, int length);
float calculate_peak_frequency(float* input_buffer, uint16_t buffer_len, uint16_t sample_rate,
                              arm_rfft_fast_instance_f32* fft_inst, float* fft_out, float* fft_magnitude);

void RMS_Average_Init(RMS_Average_TypeDef *rms_avg);
float RMS_Average_Update(RMS_Average_TypeDef *rms_avg, float* input_buffer, uint32_t length);
void RMS_Average_Clear(RMS_Average_TypeDef *rms_avg);

void Frequency_Calibration_Init(Frequency_Calibration_TypeDef *freq_cal, float target_value);
float Process_Frequency_Calibration(Frequency_Calibration_TypeDef *freq_cal,
                                   RMS_Average_TypeDef *rms_avg,
                                   float peak_frequency,
                                   float target_freq,
                                   float rms_value,
                                   uint8_t range_id);

void FFT_Processor_Init(FFT_Processor_TypeDef *fft_proc, uint32_t sample_rate);
float FFT_Calculate_Peak_Frequency(FFT_Processor_TypeDef *fft_proc, float *input_buffer);

void Noise_Calculator_Init(Noise_Calculator_TypeDef *noise_calc);
float Calculate_Noise_After_Signal_Removal(Noise_Calculator_TypeDef *noise_calc,
                                          float *original_signal,
                                          float peak_frequency,
                                          uint32_t length);

void DC_Calibration_Init(DC_Calibration_TypeDef *dc_cal, float target, float coeff);
float Process_DC_Calibration(DC_Calibration_TypeDef *dc_cal, float dc_component);

float Calculate_DC_Noise(Noise_Calculator_TypeDef *noise_calc,
                         float *signal,
                         uint32_t length,
                         float dc_value);


#endif //DATA_PROCESS_H
