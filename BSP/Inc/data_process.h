//
// Created by 23927 on 25-7-25.
//

#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H

#include "main.h"
#include "arm_math.h"

float signal_preprocess(float *input_data, int input_length, float *output_buffer, int fft_len);
void copy_float_array(const float *source, float *destination, int length);
float calculate_rms(const float *data, int length);
float calculate_peak_frequency(float *input_data, int fft_len, int sample_rate,
                              arm_rfft_fast_instance_f32 *fft_instance,
                              float *fft_output_buffer, float *fft_mag_buffer);

#endif //DATA_PROCESS_H
