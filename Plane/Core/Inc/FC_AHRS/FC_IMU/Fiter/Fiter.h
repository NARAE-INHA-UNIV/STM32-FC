/*
 * Fiter.h
 * FC_AHRS/FC_IMU/Fiter/Fiter.h
 *
 *  Created on: July 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */

#ifndef INC_FC_AHRS_FC_IMU_FITER_FITER_H_
#define INC_FC_AHRS_FC_IMU_FITER_FITER_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <FC_AHRS/FC_IMU/BMI323/BMI323.h>
#include <FC_AHRS/FC_IMU/ICM42688P/ICM42688.h>


/* Variables -----------------------------------------------------------------*/
// 필터 출력 오일러 각
extern float roll_kf;
extern float pitch_kf;
extern float yaw_kf;


/* 최종 함수 -------------------------------------------------------------------*/
void AHRS(
    int32_t x_acc_raw,  int32_t y_acc_raw,  int32_t z_acc_raw,
    int32_t x_gyro_raw, int32_t y_gyro_raw, int32_t z_gyro_raw,
    int32_t x_mag_raw,  int32_t y_mag_raw,  int32_t z_mag_raw,
    float dt);


/* 내부 함수 -------------------------------------------------------------------*/
typedef struct {
    float phy_pre;
    float alpha;
} LPFState;
void LPF_Setting(void);
float LPF(LPFState *str, float in);
float YAW(float x_mag, float y_mag, float z_mag, float roll, float pitch);


/* 행렬 계산 함수 ---------------------------------------------------------------*/
void mat_add(const float *A, const float *B, float *C, int rows, int cols);
void mat_sub(const float *A, const float *B, float *C, int rows, int cols);
void mat_mul(const float *A, const float *B, float *C, int rowsA, int colsA, int colsB);
void mat_copy(const float *A, float *C, int rows, int cols);
void mat_trp(const float *A, float *C, int rows, int cols);
int mat_inv(const float *A, float *C, int n);


#endif /* INC_FC_AHRS_FC_IMU_FITER_FITER_H_ */
