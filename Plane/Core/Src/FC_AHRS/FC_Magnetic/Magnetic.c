/*
 * Magnetic.c
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_Magnetic/Magnetic_module.h>


/* Functions -----------------------------------------------------------------*/
uint8_t MAG_Initialization(void)
{
	LIS2MDL_Initialization();
	return 0;
}


uint8_t MAG_GetData(void)
{
	LIS2MDL_GetData((SCALED_IMU*)&msg.scaled_imu);
	return 0;
}


/* Functions 2 ---------------------------------------------------------------*/
unsigned int MAG_getDataRaw(void)
{
	return 0;
}

