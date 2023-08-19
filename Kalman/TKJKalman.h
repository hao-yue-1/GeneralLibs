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

#ifndef  TKJKALMAN_H
#define TKJKALMAN_H

typedef struct
{
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
}TKJKalman;

void TKJKalmanInit(TKJKalman *handle);

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float TKJKalmanGetAngle(TKJKalman *handle, float newAngle, float newRate, float dt);

void TKJKalmanSetAngle(TKJKalman* handle, float angle); // Used to set angle, this should be set as the starting angle
float TKJKalmanGetRate(TKJKalman* handle); // Return the unbiased rate

/* These are used to tune the Kalman filter */
void TKJKalmanSetQangle(TKJKalman* handle, float Q_angle);

/**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp.
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
void TKJKalmanSetQbias(TKJKalman* handle, float Q_bias);
void TKJKalmanSetRmeasure(TKJKalman* handle, float R_measure);

float TKJKalmanGetQangle(TKJKalman* handle);
float TKJKalmanGetQbias(TKJKalman* handle);
float TKJKalmanGetRmeasure(TKJKalman* handle);

#endif
