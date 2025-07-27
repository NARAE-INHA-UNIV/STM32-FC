/*
 * Kalman.c
 *
 *  Created on: Jul 6, 2025
 *      Author: rlawn
 */
#include <FC_AHRS/AP_Filter/Kalman/Kalman.h>

// Kalman 필터 초기화
void Kalman_Init(Kalman1D_t *kalman, float Q, float R, float initial_P)
{
    kalman->Q = Q;
    kalman->R = R;
    kalman->P = initial_P;
    kalman->x = 0.0f; // 초기 상태 (예: 속도 0)
}

// Kalman 필터 업데이트 (예측 + 갱신)
float Kalman_Update(Kalman1D_t *kalman, float measured, float u, float dt)
{
    /*
     * measured: GPS 측정값 (예: GPS 속도)
     * u: IMU 입력값 (예: 가속도)
     * dt: 시간 간격
     */

    // === 예측 단계 ===
    kalman->x += u * dt; // IMU 가속도로 속도 예측
    kalman->P += kalman->Q; // 공분산 업데이트

    // === 갱신 단계 ===
    kalman->K = kalman->P / (kalman->P + kalman->R); // 칼만 이득 계산
    kalman->x += kalman->K * (measured - kalman->x); // 상태 보정
    kalman->P = (1.0f - kalman->K) * kalman->P;     // 공분산 보정

    return kalman->x;
}
// 예측 단계: IMU 가속도를 이용해 빠르게 속도를 예측
// 갱신 단계: GPS 속도로 정확하게 보정
// Kalman Gain은 자동으로 계산되어 센서 신뢰도에 따라 가중치 조절
