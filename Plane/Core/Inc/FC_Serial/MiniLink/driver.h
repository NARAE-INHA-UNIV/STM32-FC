/*
 * driver.h
 * MiniLink Driver
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERIAL_MINILINK_DRIVER_H_
#define INC_FC_SERIAL_MINILINK_DRIVER_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/MiniLink/MSG_CMD/Common.h>
#include <FC_Serial/MiniLink/MSG_CMD/Development.h>

#include <FC_Serial/MiniLink/Param/Param_type.h>


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

	// Development
	AIRSPEED airspeed1;
} Messages;

extern Messages msg;


typedef struct __attribute__((packed)){
	PARAM_HEADER header;
	INS ins;
	PARAM_RC rc;
	PARAM_SERIAL serial[4];
	PARAM_SERVO servo;
} PARAM;

extern PARAM param;


#endif /* INC_FC_SERIAL_MINILINK_DRIVER_H_ */
