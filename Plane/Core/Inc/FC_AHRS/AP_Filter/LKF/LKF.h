/*
 * LKF.h
 * FC_AHRS/AP_Filter/LKF
 *
 *  Created on: July 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */

#ifndef INC_FC_AHRS_AP_FILTER_LKF_LKF_H_
#define INC_FC_AHRS_AP_FILTER_LKF_LKF_H_


/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>

#include <FC_Basic/Matrix/Matrix.h>

#include <FC_AHRS/AP_Filter/LKF/driver.h>

#include <FC_Serial/MiniLink/driver.h>


/* Variables -----------------------------------------------------------------*/
// 필터 출력 오일러 각
extern float roll_kf;
extern float pitch_kf;
extern float yaw_kf;




/* Variables -----------------------------------------------------------------*/
typedef struct {
    float phy_pre;
    float alpha;
} LPFState;


/* Functions -----------------------------------------------------------------*/
void LPF_init(void);
float LPF_update(LPFState *str, float in);
float YAW(float x_mag, float y_mag, float z_mag, float roll, float pitch);


/* 행렬 계산 함수 ---------------------------------------------------------------*/
void mat_add(const float *A, const float *B, float *C, int rows, int cols);
void mat_sub(const float *A, const float *B, float *C, int rows, int cols);
void mat_mul(const float *A, const float *B, float *C, int rowsA, int colsA, int colsB);
void mat_copy(const float *A, float *C, int rows, int cols);
void mat_trp(const float *A, float *C, int rows, int cols);
int mat_inv(const float *A, float *C, int n);


#endif /* INC_FC_AHRS_AP_FILTER_LKF_LKF_H_ */
