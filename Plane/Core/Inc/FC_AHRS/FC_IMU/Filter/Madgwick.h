/*
 * Madgwick.h
 *
 *  Created on: Jul 3, 2025
 *      Author: rlawn
 */

#ifndef INC_FC_AHRS_FC_IMU_FILTER_MADGWICK_H_
#define INC_FC_AHRS_FC_IMU_FILTER_MADGWICK_H_


#include <math.h>
#include <GCS_MiniLink/GCS_MiniLink.h>


void Madgwick_update(SCALED_IMU* imu);
void Madgwick_GetEuler(float* roll, float* pitch, float* yaw);


#endif /* INC_FC_AHRS_FC_IMU_FILTER_MADGWICK_H_ */
