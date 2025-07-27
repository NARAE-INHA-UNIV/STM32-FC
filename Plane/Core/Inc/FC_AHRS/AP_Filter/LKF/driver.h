/*
 * LKF/driver.h
 *
 *  Created on: Jul 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */

#ifndef INC_FC_AHRS_AP_FILTER_LKF_DRIVER_H_
#define INC_FC_AHRS_AP_FILTER_LKF_DRIVER_H_

#include <FC_Serial/MiniLink/driver.h>


void LKF_Update(SCALED_IMU* imu, float dt);


#endif /* INC_FC_AHRS_AP_FILTER_LKF_DRIVER_H_ */
