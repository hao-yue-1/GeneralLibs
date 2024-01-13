//
// Created by yue on 2023/12/10.
//

#include "lowpass_filter.h"
#include <math.h>

/**
 * 配置一阶低通滤波器的参数
 * @param handle 一阶低通滤波器结构体句柄
 * @param cutoff_freq 截止频率(Hz)
 * @param sample_freq 采样屏幕(Hz)
 */
void LowPassFilter_Config(LowPassFilter_t* handle, float cutoff_freq, float sample_freq)
{
    float sample_period = 1.0f / sample_freq;   // 采样周期(s)

    /* k = Ts/(Ts+(1/2πfc)) */
    handle->k = sample_period / (sample_period + 1.0f / (2 * 3.1415926f * cutoff_freq));
}

/**
 * 根据输入更新一阶低通滤波器的输出
 * @param handle 一阶低通滤波器结构体句柄
 * @param value 输入值
 */
float LowPassFilter_UpdateValue(LowPassFilter_t* handle, float value)
{
    /* f(n) = k*x(n)+(1-k)*f(n-1) */
    handle->curr_value = handle->k * value + (1 - handle->k) * handle->last_value;

    return handle->curr_value;
}
