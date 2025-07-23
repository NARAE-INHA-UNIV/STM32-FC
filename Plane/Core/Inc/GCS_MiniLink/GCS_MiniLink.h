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
#include <GCS_MiniLink/Common_MSG.h>
#include <stdint.h>


typedef struct __attribute__((packed)){
	uint8_t stx;
	uint8_t length;
	uint8_t seq;
	uint16_t msgId;
} MiniLinkHeader;


typedef struct __attribute__((packed)){
	MiniLinkHeader header;
	uint8_t* payload;
	struct{
		uint8_t ack : 1;
	} flag;
} MiniLinkPacket;


typedef struct __attribute__((packed)){
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
} Common;


extern Common msg;
/* Variables -----------------------------------------------------------------*/



#endif /* INC_GCS_MAVLINK_GCS_COMMON_H_ */
