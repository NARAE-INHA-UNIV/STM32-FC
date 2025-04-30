/*
 * driver_IMU.c
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_IMU/driver_IMU.h>
#include <GCS_MAVLink/GCS_Common.h>


/* Functions -----------------------------------------------------------------*/
int IMU_Initialization(void)
{
	ICM42688_Initialization();
	return 0;
}


void IMU_GetData(void)
{

	// SCALED_IMU
	ICM42688_GetData();

	// SCALED_IMU2
	// SCALED_IMU3
	return;
}
