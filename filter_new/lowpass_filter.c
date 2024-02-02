//
// Created by yue on 2023/12/10.
//

#include "lowpass_filter.h"
#include <math.h>

/**
 * ����һ�׵�ͨ�˲����Ĳ���
 * @param handle һ�׵�ͨ�˲����ṹ����
 * @param cutoff_freq ��ֹƵ��(Hz)
 * @param sample_freq ������Ļ(Hz)
 */
void LowPassFilter_Config(LowPassFilter_t* handle, float cutoff_freq, float sample_freq)
{
    float sample_period = 1.0f / sample_freq;   // ��������(s)

    /* k = Ts/(Ts+(1/2��fc)) */
    handle->k = sample_period / (sample_period + 1.0f / (2 * 3.1415926f * cutoff_freq));
}

/**
 * �����������һ�׵�ͨ�˲��������
 * @param handle һ�׵�ͨ�˲����ṹ����
 * @param value ����ֵ
 */
float LowPassFilter_UpdateValue(LowPassFilter_t* handle, float value)
{
    /* f(n) = k*x(n)+(1-k)*f(n-1) */
    handle->curr_value = handle->k * value + (1 - handle->k) * handle->last_value;

    return handle->curr_value;
}
