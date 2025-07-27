/*
 * FC_Baro/Baro.h
 *
 *  Created on: June 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_BARO_BARO_H_
#define INC_FC_BARO_BARO_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_Baro/Baro.h>
#include <FC_AHRS/FC_Baro/LPS22HH/LPS22HH.h>
#include <FC_Serial/MiniLink/MiniLink_module.h>
#include "main.h"
#include <math.h>



/* Variables -----------------------------------------------------------------*/
extern SCALED_PRESSURE scaled_pressure;


/* Functions 2 ---------------------------------------------------------------*/
float pascal2meter(float pressure, float temperature);


#endif
