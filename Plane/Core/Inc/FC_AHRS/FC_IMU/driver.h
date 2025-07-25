/*
 * FC_IMU/driver.h
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_DRIVER_H_
#define INC_FC_IMU_DRIVER_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Functions -----------------------------------------------------------------*/
uint8_t IMU_Initialization(void);
uint8_t IMU_GetData(void);
void IMU_CalibrateOffset(void);


#endif /* INC_SEN_ICM42688_DRIVER_H_ */
