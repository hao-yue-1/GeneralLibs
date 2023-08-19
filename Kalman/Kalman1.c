/*
* 简介: 一阶卡尔曼滤波算法
* 作者: WBN
* 时间: 2022/11/7
*/

#include "Kalman1.h"

/*
 *******************************************************************************************
 ** 函数功能: 初始化一阶卡尔曼结构体的参数
 ** 参    数: 1. *handle: 一阶卡尔曼滤波器结构体
			  2. _Q:	  Q参数
			  3. _R:	  R参数
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void Kalman1Config(Kalman1* handle, float _Q, float _R)
{
	handle->x = 0;
	handle->x_ = 0;
	handle->p = 0;
	handle->p_ = 0;
	handle->K = 1;
	handle->A = 1;
	handle->H = 1;
	handle->Q = _Q;
	handle->R = _R;
}

/*
 *******************************************************************************************
 ** 函数功能: 一阶卡尔曼滤波器
 ** 参    数: 1. *handle: 一阶卡尔曼滤波器结构体
			  2. value:   需要进行滤波的值
 ** 返 回 值: 滤波后的值
 ** 作    者: WBN
 ********************************************************************************************
 */ 
float Kalman1Filter(Kalman1 *handle,float value)
{
	//计算：先验估计
	handle->x_ = handle->A * handle->x_;	
	//计算：先验误差协方差
	handle->p_ = handle->A * handle->A * handle->p + handle->Q;
	//计算：卡尔曼增益
	handle->K = (handle->p_ * handle->H) / (handle->H * handle->p_ * handle->H + handle->R);
	//计算：后验估计
	handle->x = handle->x_ + handle->K * (value - handle->H * handle->x_);
	//更新：后验误差协方差
	handle->p = (1 - handle->K * handle->H) * handle->p_;
	
	return handle->x;
}
