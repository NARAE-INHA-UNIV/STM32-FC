/*
 * LIS2MDL.h
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_MAGNETIC_LIS2MDL_LIS2MDL_H_
#define INC_FC_AHRS_FC_MAGNETIC_LIS2MDL_LIS2MDL_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Functions -----------------------------------------------------------------*/
uint8_t LIS2MDL_Initialization(void);
uint8_t LIS2MDL_GetData(SCALED_IMU* imu);


#endif /* INC_FC_AHRS_FC_MAGNETIC_LIS2MDL_LIS2MDL_H_ */
