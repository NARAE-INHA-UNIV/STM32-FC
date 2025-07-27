/*
 * AHRS.h
 *
 *  Created on: Jul 23, 2025
 *      Author: rlawn, leecurrent04
 *      Email : (rlawn)
 *      		leecurrent04@inha.edu (leecurrent04)
 */

#ifndef INC_FC_AHRS_AHRS_MODULE_H_
#define INC_FC_AHRS_AHRS_MODULE_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AHRS.h>
#include <math.h>
#include <FC_AHRS/AHRS_common.h>

#include <FC_AHRS/AP_Filter/Filter.h>
#include <FC_AHRS/FC_Baro/Baro.h>
#include <FC_AHRS/FC_IMU/IMU.h>
#include <FC_AHRS/FC_Magnetic/Magnectic.h>
#include <FC_Basic/LED/LED.h>

#include <FC_Serial/MiniLink/MiniLink_module.h>


/* Variables -----------------------------------------------------------------*/

typedef struct {
    float vx;
    float vy;
    float vz;
} IMU_Velocity_t;

extern IMU_Velocity_t imu_velocity;
extern float imu_roll;
extern float imu_pitch;


/* Functions -----------------------------------------------------------------*/
void AHRS_computeVelocity(float dt);
float AHRS_calculateYAW(Vector3D mag, Euler angle);



#endif /* INC_FC_AHRS_AHRS_MODULE_H_ */
