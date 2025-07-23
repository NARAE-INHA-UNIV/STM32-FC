/*
 * FC_IMU/driver_IMU.h
 *
 *  Created on: May 1, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_IMU_H_
#define INC_FC_IMU_IMU_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <FC_AHRS/FC_IMU/driver.h>

#include <FC_AHRS/FC_IMU/ICM42688P/driver.h>
#include <FC_AHRS/FC_IMU/BMI323/driver.h>

#include <FC_AHRS/FC_IMU/Madgwick.h>
#include <GCS_MiniLink/GCS_MiniLink.h>


/* Variables -----------------------------------------------------------------*/
extern SCALED_IMU scaled_imu;
extern RAW_IMU raw_imu;


typedef struct {
    float vx;
    float vy;
    float vz;
} IMU_Velocity_t;

/* extern 변수 선언 */

extern IMU_Velocity_t imu_velocity;
extern float imu_roll;
extern float imu_pitch;


/* Functions -----------------------------------------------------------------*/
unsigned int IMU_getDataRaw(void);
void IMU_computeVelocity(float dt);


void KalmanFilter(void);
void ComplementaryFilter(void);


#endif /* INC_FC_IMU_IMU_H_ */
