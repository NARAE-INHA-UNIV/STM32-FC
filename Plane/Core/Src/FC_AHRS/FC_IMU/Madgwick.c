/*
 * Madgwick.c
 *
 *  Created on: Jul 3, 2025
 *      Author: rlawn
 */

#include <FC_AHRS/FC_IMU/Madgwick.h>
#include <math.h>
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 쿼터니언 초기값
float beta = 0.1f;                                       // 필터 게인 (응답 속도 조절)

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;


    // 가속도 normalize
    recipNorm = sqrtf(ax * ax + ay * ay + az * az); // 벡터 크기 계산
    if (recipNorm == 0.0f) return;                 // 크기 0이면 리턴
    recipNorm = 1.0f / recipNorm;                  // 역수 계산
    ax *= recipNorm;                               // 정규화된 X
    ay *= recipNorm;                               // 정규화된 Y
    az *= recipNorm;                               // 정규화된 Z

    // 쿼터니언 미분 (자이로 기반)
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // gradient descent 보정항 (가속도 기반) 경사하강법
    s0 = _2q2 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2 * q3 - ay);
    s1 = _2q3 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2 * q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2 * q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az);
    s3 = _2q1 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2 * q3 - ay);

    recipNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 보정항 정규화
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 쿼터니언 미분 보정
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    // 쿼터니언 이분 적분 (dt=0.005 기준)
    q0 += qDot1 * 0.005;
    q1 += qDot2 * 0.005;
    q2 += qDot3 * 0.005;
    q3 += qDot4 * 0.005;

    // normalize
    recipNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    recipNorm = 1.0f / recipNorm;
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Madgwick_GetEuler(float* roll, float* pitch, float* yaw)
{
    *roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI; // Roll 계산
    *pitch = asinf(2.0f * (q0*q2 - q3*q1)) * 180.0f / M_PI;                                // Pitch 계산
    *yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI; // Yaw 계산
}

