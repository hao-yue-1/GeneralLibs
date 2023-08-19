/*
* 简介: 一阶卡尔曼滤波算法
* 作者: WBN
* 时间: 2022/11/7
*/

#ifndef KALMAN_H
#define KALMAN_H

typedef struct
{
	float x_;	//先验估计
	float x;	//后验估计
	float p_;	//先验误差协方差
	float p;	//后验误差协方差
	float K;	//卡尔曼增益
	float A;	//过程系数=1
	float Q;	//过程噪声方差
	float R;	//测量噪声方差
	float H;	//测量系数=1

}Kalman1;

void Kalman1Config(Kalman1* handle, float _Q, float _R);
float Kalman1Filter(Kalman1 *handle,float value);

#endif
