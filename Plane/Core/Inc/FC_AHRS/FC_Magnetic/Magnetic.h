/*
 * Magnetic.h
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_MAGNETIC_MAGNETIC_H_
#define INC_FC_AHRS_FC_MAGNETIC_MAGNETIC_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <FC_Basic/SPI.h>

#include <FC_AHRS/FC_Magnetic/driver.h>
#include <FC_AHRS/FC_Magnetic/LIS2MDL/driver.h>

#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
extern SCALED_IMU scaled_imu;
extern RAW_IMU raw_imu;


/* Functions -----------------------------------------------------------------*/
unsigned int MAG_getDataRaw(void);


#endif /* INC_FC_AHRS_FC_MAGNETIC_MAGNETIC_H_ */
