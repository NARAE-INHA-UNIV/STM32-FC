/*
 * Filter.h
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_AP_FILTER_FILTER_H_
#define INC_FC_AHRS_AP_FILTER_FILTER_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AP_Filter/Kalman.h>
#include <FC_AHRS/AP_Filter/Madgwick.h>
//#include <FC_AHRS/AP_Filter/MadgwickAHRS.h>


/* Variables -----------------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
void KalmanFilter(void);
void ComplementaryFilter(void);


#endif /* INC_FC_AHRS_AP_FILTER_FILTER_H_ */
