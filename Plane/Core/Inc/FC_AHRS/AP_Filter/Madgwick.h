/*
 * Madgwick.h
 *
 *  Created on: Jul 3, 2025
 *      Author: rlawn
 */

#ifndef INC_FC_AHRS_AP_FILTER_MADGWICK_H_
#define INC_FC_AHRS_AP_FILTER_MADGWICK_H_


/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include <FC_Serial/MiniLink/MiniLink.h>


/* Functions -----------------------------------------------------------------*/
void Magwick_Initialization(ATTITUDE_QUATERNION* qu);
void Madgwick_Update(ATTITUDE_QUATERNION* qu, SCALED_IMU* imu);
void Madgwick_GetEuler(ATTITUDE_QUATERNION* qu, ATTITUDE* atti);


#endif /* INC_FC_AHRS_AP_FILTER_MADGWICK_H_ */
