/*
* 简介: IMU的姿态融合解算 通过六轴数据计算四元数 再转换为欧拉角
* 作者: WBN
* 时间: 2022/5/11
*/

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>		//int16_t

typedef struct 
{
    float pitch;//欧拉角
    float roll;
    float yaw;
    float q0;   //四元数
    float q1;
    float q2;
    float q3;
    float kp;   //加速度计收敛速率的比例增益
    float ki;   //陀螺仪收敛速率的积分增益
    float i_err_x;  //误差积分
    float i_err_y;
    float i_err_z;
    float values[6];   //0~2: ax~az     3~5: gx~gz	//原作者给了10的数组长度，但是程序中只需要用到6位
}EulerianQuater;

typedef struct
{
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
}GyroAcc;

void EulerianQuaterInit(EulerianQuater* handle, float _kp, float _ki);
void GetEulerianAngles(EulerianQuater* handle, GyroAcc data, float dt);

#endif //ICM20602_ATTITUDE_H
