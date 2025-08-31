/*
 * IMU.h
 *
 *  Created on: May 1, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_IMU_H_
#define INC_FC_IMU_IMU_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include <FC_AHRS/FC_IMU/IMU.h>

#include <FC_AHRS/FC_IMU/ICM42688P/ICM42688P.h>
#include <FC_AHRS/FC_IMU/BMI323/BMI323.h>

#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
extern SCALED_IMU scaled_imu;
extern RAW_IMU raw_imu;


/* Functions -----------------------------------------------------------------*/
unsigned int IMU_getDataRaw(void);


#endif /* INC_FC_IMU_IMU_H_ */
