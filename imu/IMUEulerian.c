/*
* 简介: IMU的姿态融合解算 通过六轴数据计算四元数 再转换为欧拉角
* 作者: WBN
* 时间: 2022/5/11
*/

#include "IMUeulerian.h"
#include <math.h>

#define M_PI 3.14159265358979323846f    //与math.h中原有的定义重复

/*
 *******************************************************************************************
 ** 函数功能: 初始化结构体参数
 ** 参    数: 1. *handle: 结构体
			  2. _Q:	  加速度计收敛速率的比例增益
			  3. _R:	  陀螺仪收敛速率的积分增益
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */ 
void EulerianQuaterInit(EulerianQuater* handle, float _kp, float _ki)
{
    handle->pitch = 0;
    handle->roll = 0;
    handle->yaw = 0;
    handle->q0 = 1;
    handle->q1 = 0;
    handle->q2 = 0;
    handle->q3 = 0;
    handle->kp = _kp;   //一般给50
    handle->ki = _ki;   //一般给0.2
    handle->i_err_x = 0;
    handle->i_err_y = 0;
    handle->i_err_z = 0;
    for (int i = 0; i < 6; i++)
    {
        handle->values[i] = 0;
    }
}

/*
 *******************************************************************************************
 ** 函数功能: 归一化处理
 ** 参    数: x: 待处理数据
 ** 返 回 值: 归一化系数
 ** 作    者: WBN
 ********************************************************************************************
 */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*
 *******************************************************************************************
 ** 函数功能: 对六轴数据处理: 对加速度计一阶低通滤波 对陀螺仪进行单位转换(弧度每秒: 2000dps)
 ** 参    数: 1. *value: 存储六轴数据的数组
              2. data:   陀螺仪和加速度计的原始数据
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
#define new_weight           0.35f
#define old_weight           0.65f
void IMUGetValues(GyroAcc data, float* values)
{
    static double lastaccel[3] = { 0,0,0 };
    int i;
    //accel
    values[0] = ((float)data.acc_x) * new_weight + lastaccel[0] * old_weight;
    values[1] = ((float)data.acc_y) * new_weight + lastaccel[1] * old_weight;
    values[2] = ((float)data.acc_z) * new_weight + lastaccel[2] * old_weight;
    for (i = 0; i < 3; i++)
    {
        lastaccel[i] = values[i];
    }
    //gyro
    values[3] = ((float)data.gyro_x) * M_PI / 180 / 16.4f;
    values[4] = ((float)data.gyro_y) * M_PI / 180 / 16.4f;
    values[5] = ((float)data.gyro_z) * M_PI / 180 / 16.4f;
}

/*
 *******************************************************************************************
 ** 函数功能: 姿态融合解算 计算四元数 使用Crazepony和核心算法 互补滤波 无Kalman滤波
 ** 参    数: 1. *handle: 结构体
			  2. dt: 采样时间间隔 单位 s
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
static void IMUGetQuater(EulerianQuater* handle, float dt, float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * dt;
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
    //获取四元数
    float q0 = handle->q0;
    float q1 = handle->q1;
    float q2 = handle->q2;
    float q3 = handle->q3;
    //四元数相关计算，两两相乘
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    //对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax * ax + ay * ay + az * az);    //归一化系数
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //当前的机体坐标系上的重力单位向量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //四元数计算值与加速度计测量值的误差
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    //用叉乘误差来做PI修正陀螺零偏，
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    //误差积分
    handle->i_err_x += dt * ex;   // integral error scaled by Ki
    handle->i_err_y += dt * ey;
    handle->i_err_z += dt * ez;
    //角速度修正
    gx = gx + handle->kp * ex + handle->ki * handle->i_err_x;
    gy = gy + handle->kp * ey + handle->ki * handle->i_err_y;
    gz = gz + handle->kp * ez + handle->ki * handle->i_err_z;
    /*数据修正完成，下面是四元数微分方程*/
    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;
    //标准化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    handle->q0 = q0 * norm;
    handle->q1 = q1 * norm;
    handle->q2 = q2 * norm;
    handle->q3 = q3 * norm;
}

/*
 *******************************************************************************************
 ** 函数功能: 将四元数转换为欧拉角
 ** 参    数: 1. *handle: 结构体
			  2. data:    陀螺仪和加速度计的原始数据
			  3. dt:      采样时间间隔 单位 s
 ** 返 回 值: 无
 ** 作    者: WBN
 ********************************************************************************************
 */
void GetEulerianAngles(EulerianQuater* handle, GyroAcc data, float dt)
{
    IMUGetValues(data,handle->values);  //对加速度计低通，对陀螺仪单位换算
    IMUGetQuater(handle, dt, handle->values[3], handle->values[4], handle->values[5], handle->values[0], handle->values[1], handle->values[2]);    //姿态解算融合
    //四元数
    float q0 = handle->q0;
    float q1 = handle->q1;
    float q2 = handle->q2;
    float q3 = handle->q3;
    //计算欧拉角
    handle->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI;
    handle->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI;
    handle->yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;

    //姿态限度的限制
//    if(handle->roll>90 || handle->roll<-90)
//    {
//        if(handle->pitch > 0)
//        {
//            handle->pitch = 180-handle->pitch;
//        }
//        if(handle->pitch < 0)
//        {
//            handle->pitch = -(180+handle->pitch);
//        }
//    }
//    if(handle->yaw > 180)
//    {
//        handle->yaw -=360;
//    }
//    else if(handle->yaw <-180)
//    {
//        handle->yaw +=360;
//    }
}
