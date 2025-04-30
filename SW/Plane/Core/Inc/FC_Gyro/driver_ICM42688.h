/*
 * FC_Gyro/driver_ICM42688.h
 *
 *  Created on: Mar 8, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_GYRO_DRIVER_ICM42688_H_
#define INC_FC_GYRO_DRIVER_ICM42688_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <GCS_MAVLink/GCS_Common.h>


/* Variables -----------------------------------------------------------------*/
extern RAW_IMU raw_imu;
extern SCALED_IMU scaled_imu;

extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;


/* Functions 1 ---------------------------------------------------------------*/
int ICM42688_Initialization(void);
int ICM42688_GetData(void);


/* Functions 2 ---------------------------------------------------------------*/
void ICM42688_Get6AxisRawData(void);
void ICM42688_raw2dps(void);
//void ICM42688_Get3AxisGyroRawData(short* gyro);
//void ICM42688_Get3AxisAccRawData(short* accel);
//int ICM42688_DataReady(void);


#endif /* INC_SEN_ICM42688_DRIVER_H_ */
