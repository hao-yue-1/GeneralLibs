/*
* 简介: 将TKJ Electronics开源的基于C++卡尔曼算法整理为C语言 方便移植到其他嵌入式平台
*       未对原算法做任何修改 且保留TKJ Electronics的英文注释
*       TKJ Electronics开源链接: https://github.com/TKJElectronics/KalmanFilter
* 作者: WBN
* 时间: 2022/11/7
*/

/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "TKJKalman.h"

void TKJKalmanInit(TKJKalman* handle)
{
    /* We will set the variables like so, these can also be tuned by the user */
    handle->Q_angle = 0.001f;
    handle->Q_bias = 0.003f;
    handle->R_measure = 0.03f;

    handle->angle = 0.0f; // Reset the angle
    handle->bias = 0.0f; // Reset bias

    handle->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    handle->P[0][1] = 0.0f;
    handle->P[1][0] = 0.0f;
    handle->P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float TKJKalmanGetAngle(TKJKalman* handle, float newAngle, float newRate, float dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    handle->rate = newRate - handle->bias;
    handle->angle += dt * handle->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    handle->P[0][0] += dt * (dt * handle->P[1][1] - handle->P[0][1] - handle->P[1][0] + handle->Q_angle);
    handle->P[0][1] -= dt * handle->P[1][1];
    handle->P[1][0] -= dt * handle->P[1][1];
    handle->P[1][1] += handle->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = handle->P[0][0] + handle->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = handle->P[0][0] / S;
    K[1] = handle->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - handle->angle; // Angle difference
    /* Step 6 */
    handle->angle += K[0] * y;
    handle->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = handle->P[0][0];
    float P01_temp = handle->P[0][1];

    handle->P[0][0] -= K[0] * P00_temp;
    handle->P[0][1] -= K[0] * P01_temp;
    handle->P[1][0] -= K[1] * P00_temp;
    handle->P[1][1] -= K[1] * P01_temp;

    return handle->angle;
};

// Used to set angle, this should be set as the starting angle
void TKJKalmanSetAngle(TKJKalman* handle, float angle)
{
    handle->angle = angle;
}

// Return the unbiased rate
float TKJKalmanGetRate(TKJKalman* handle)
{
    return handle->rate;
}

/* These are used to tune the Kalman filter */
void TKJKalmanSetQangle(TKJKalman* handle, float Q_angle)
{
    handle->Q_angle = Q_angle;
}

void TKJKalmanSetQbias(TKJKalman* handle, float Q_bias)
{ 
    handle->Q_bias = Q_bias;
}

void TKJKalmanSetRmeasure(TKJKalman* handle, float R_measure)
{ 
    handle->R_measure = R_measure;
}

float TKJKalmanGetQangle(TKJKalman* handle)
{ 
    return handle->Q_angle;
}

float TKJKalmanGetQbias(TKJKalman* handle)
{
    return handle->Q_bias;
}

float TKJKalmanGetRmeasure(TKJKalman* handle)
{
    return handle->R_measure;
}
