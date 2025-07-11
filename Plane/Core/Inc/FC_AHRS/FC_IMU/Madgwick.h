/*
 * Madgwick.h
 *
 *  Created on: Jul 3, 2025
 *      Author: rlawn
 */

#ifndef INC_FC_AHRS_FC_IMU_MADGWICK_H_
#define INC_FC_AHRS_FC_IMU_MADGWICK_H_

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az); // Madgwick 필터 메인 업데이트 함수
void Madgwick_GetEuler(float* roll, float* pitch, float* yaw);                         // 최종 Roll, Pitch, Yaw 추출 함수

#endif /* INC_FC_AHRS_FC_IMU_MADGWICK_H_ */
