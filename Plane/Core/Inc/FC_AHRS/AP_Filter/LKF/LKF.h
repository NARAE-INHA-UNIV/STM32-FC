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

#include <FC_AHRS/AHRS_common.h>

#include <FC_AHRS/AP_Filter/LKF/driver.h>

#include <FC_Serial/MiniLink/driver.h>


/* Variables -----------------------------------------------------------------*/
// 필터 출력 오일러 각
extern float roll_kf;
extern float pitch_kf;
extern float yaw_kf;




/* Variables -----------------------------------------------------------------*/
typedef struct {
	Vector3D previous;
	Vector3D alpha;
} LPFState;



/* Functions -----------------------------------------------------------------*/
void LPF_init(void);
Vector3D LPF_update3D(LPFState *t0, Vector3D* in);
float LPF_update(float alpha, float* previous, float in);
float calculate_YAW(Vector3D mag, Euler angle);


#endif /* INC_FC_AHRS_AP_FILTER_LKF_LKF_H_ */
