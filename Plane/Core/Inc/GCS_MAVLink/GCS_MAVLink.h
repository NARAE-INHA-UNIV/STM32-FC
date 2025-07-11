/*
 * GCS_MAVLink.h
 *
 *  Created on: Mar 27, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_GCS_MAVLINK_GCS_MAVLink_H_
#define INC_GCS_MAVLINK_GCS_MAVLink_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>

#include <GCS_MAVLink/MAVLink_Common_MSG.h>


typedef struct __attribute__((packed)){
	SYSTEM_TIME system_time;
	SCALED_IMU scaled_imu;	//26 가속도계하고 자이로 합친 값이 출력될 예쩡
	RAW_IMU raw_imu;				// 27
	SCALED_PRESSURE scaled_pressure;		// 29
	SERVO_OUTPUT_RAW servo_output_raw; //36
	RC_CHANNELS RC_channels;  //65
	SCALED_IMU2 scaled_imu2;			// 116
} Common;

extern Common msg;
/* Variables -----------------------------------------------------------------*/



#endif /* INC_GCS_MAVLINK_GCS_COMMON_H_ */
