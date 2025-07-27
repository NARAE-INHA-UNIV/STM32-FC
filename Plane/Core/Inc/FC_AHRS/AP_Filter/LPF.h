/*
 * LPF.h
 *
 *  Created on: Jul 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */

#ifndef INC_FC_AHRS_AP_FILTER_LPF_H_
#define INC_FC_AHRS_AP_FILTER_LPF_H_


/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdint.h>

#include <FC_AHRS/AHRS_common.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
typedef struct {
	Vector3D previous;
	Vector3D alpha;
} LPFState;

extern LPFState lpf_acc;
extern LPFState lpf_gyro;
extern LPFState lpf_mag;


/* Functions -----------------------------------------------------------------*/
void LPF_init(void);
Vector3D LPF_update3D(LPFState *t0, Vector3D* in);
float LPF_update(float alpha, float* previous, float in);


#endif /* INC_FC_AHRS_AP_FILTER_LPF_H_ */
