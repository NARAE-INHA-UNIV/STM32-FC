/*
 * BMI323/driver.h
 * FC_AHRS/FC_IMU/BMI323/driver.h
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_IMU_BMI323_BMI323_H_
#define INC_FC_AHRS_FC_IMU_BMI323_BMI323_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Functions -----------------------------------------------------------------*/
uint8_t BMI323_Initialization(SCALED_IMU* p);
uint8_t BMI323_GetData(void);


#endif /* INC_FC_AHRS_FC_IMU_BMI323_BMI323_H_ */
