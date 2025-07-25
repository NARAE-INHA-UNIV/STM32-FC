/*
 * Kalman.h
 *
 *  Created on: Jul 6, 2025
 *      Author: rlawn
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"

typedef struct {
    float x;  // 상태 추정값 (예: 속도)
    float P;  // 상태 추정 오차 공분산
    float Q;  // 프로세스 잡음 공분산 (모델 신뢰도, IMU 신뢰도)
    float R;  // 측정 잡음 공분산 (측정 신뢰도, GPS 신뢰도)
    float K;  // 칼만 이득 (자동 계산되는 혼합 비율)
} Kalman1D_t;

void Kalman_Init(Kalman1D_t *kalman, float Q, float R, float initial_P);
float Kalman_Update(Kalman1D_t *kalman, float measured, float u, float dt);

#endif /* INC_KALMAN_H_ */
