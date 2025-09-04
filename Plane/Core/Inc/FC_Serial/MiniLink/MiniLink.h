/*
 * MiniLink.h
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERIAL_MINILINK_MINILINK_H_
#define INC_FC_SERIAL_MINILINK_MINILINK_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/MiniLink/MSG_CMD/Common.h>
#include <FC_Serial/MiniLink/MSG_CMD/Development.h>

#include <FC_Serial/MiniLink/Param/Param_type.h>
#include <FC_Serial/MiniLink/Param/Enum.h>


/* Variables -----------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	// Common
	SYSTEM_TIME system_time;					// 2
	SCALED_IMU scaled_imu;						// 26
	RAW_IMU raw_imu;							// 27
	SCALED_PRESSURE scaled_pressure;			// 29
	ATTITUDE attitude;							// 30
	ATTITUDE_QUATERNION attitude_quaternion;	// 31
	LOCAL_POSITION_NED local_position_ned;		// 32
	SERVO_OUTPUT_RAW servo_output_raw;			// 36
	RC_CHANNELS RC_channels;					// 65
	SCALED_IMU2 scaled_imu2;					// 116
	SCALED_IMU3 scaled_imu3;					// 129

	// Development
	AIRSPEED airspeed1;
} Messages;

extern Messages msg;


extern PARAM param;


#endif /* INC_FC_SERIAL_MINILINK_MINILINK_H_ */
