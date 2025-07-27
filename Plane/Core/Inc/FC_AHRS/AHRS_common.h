/*
 * AHRS_common.h
 *
 *  Created on: Jul 27, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_AHRS_COMMON_H_
#define INC_FC_AHRS_AHRS_COMMON_H_


/* Includes ------------------------------------------------------------------*/
#include <math.h>


/* Variables -----------------------------------------------------------------*/
typedef struct {
    float x;
    float y;
    float z;
} Vector3D;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Euler;

typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
} Quaternion;


/* Functions -----------------------------------------------------------------*/
#define DEG2RAD(x) (x * (M_PI / 180.0))
#define RAD2DEG(x) (x * (180.0 / M_PI))

Quaternion AHRS_Euler2Quaternion(const Euler ori);
Euler AHRS_Quaternion2Euler(const Quaternion ori);
Quaternion AHRS_NormalizeQuaternion(const Quaternion ori);


#endif /* INC_FC_AHRS_AHRS_COMMON_H_ */
