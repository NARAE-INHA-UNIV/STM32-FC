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
#include <main.h>


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
	SYSTEM_TIME system_time;
	SCALED_IMU scaled_imu;
	RAW_IMU raw_imu;				// 27
	SCALED_PRESSURE scaled_pressure;		// 29
	SERVO_OUTPUT_RAW servo_output_raw;
	RC_CHANNELS RC_channels;
	SCALED_IMU2 scaled_imu2;			// 116
} Common;

extern Common msg;
/* Variables -----------------------------------------------------------------*/



#endif /* INC_GCS_MAVLINK_GCS_COMMON_H_ */
