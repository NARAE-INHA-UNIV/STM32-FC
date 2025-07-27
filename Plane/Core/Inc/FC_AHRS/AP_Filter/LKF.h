/*
 * LKF.h
 * FC_AHRS/AP_Filter/LKF
 *
 *  Created on: July 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */

#ifndef INC_FC_AHRS_AP_FILTER_LKF_H_
#define INC_FC_AHRS_AP_FILTER_LKF_H_


/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>

#include <FC_AHRS/AHRS_common.h>
#include <FC_Basic/Matrix/Matrix.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
Quaternion LKF_Update(Vector3D gyro, Quaternion angQ, float dt);


#endif /* INC_FC_AHRS_AP_FILTER_LKF_H_ */
