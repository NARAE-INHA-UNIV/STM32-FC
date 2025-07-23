/*
 * Speedmeter.h
 *
 *  Created on: Jul 11, 2025
 *      Author: rlawn
 */

#ifndef __SPEEDMETER_H__
#define __SPEEDMETER_H__

#include "main.h"

void Speedmeter_Init(void);
void Speedmeter_Calibrate(void);
void Speedmeter_Update(void);
float Speedmeter_GetAirspeed(void);
void Speedmeter_Filter(float raw_airspeed);

#endif
 /* INC_SPEEDMETER_H_ */
