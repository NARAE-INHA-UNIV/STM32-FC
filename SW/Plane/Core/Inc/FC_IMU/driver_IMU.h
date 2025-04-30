/*
 * FC_IMU/driver_IMU.h
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_DRIVER_IMU_H_
#define INC_FC_IMU_DRIVER_IMU_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <GCS_MAVLink/GCS_Common.h>

#include <FC_Gyro/driver_ICM42688.h>


/* Variables -----------------------------------------------------------------*/
extern SCALED_IMU scaled_imu;
extern RAW_IMU raw_imu;


/* Functions -----------------------------------------------------------------*/
int IMU_Initialization(void);
void IMU_GetData(void);


#endif /* INC_SEN_ICM42688_DRIVER_H_ */
