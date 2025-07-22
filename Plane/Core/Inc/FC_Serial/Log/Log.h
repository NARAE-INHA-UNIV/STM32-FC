/*
 * Log.h
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_LOG_LOG_H_
#define INC_FC_LOG_LOG_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <stdlib.h>
#include <string.h>

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <FC_Param/Param.h>

/* Variables -----------------------------------------------------------------*/

typedef struct{
	uint8_t* start;
	uint8_t* offset;
	uint16_t length;
} JumboPakcet;

extern JumboPakcet jumboTx;

/* Functions -----------------------------------------------------------------*/
int Log_Send();

/* Functions 1 ---------------------------------------------------------------*/
int Log_pack(uint16_t msgId, uint8_t* payload, uint8_t len);
int Log_transmit_UART(uint8_t *packet, uint8_t len);
int Log_addMailBox_CDC(uint8_t* packet, uint8_t len);
int Log_transmit_CDC();


/* Macros --------------------------------------------------------------------*/
#define LOG_MAVLINK_HEADER 0xFA
#define LOG_BUFFER_SIZE 1024

#define LOG_TABLE(X) \
    X(26,  scaled_imu) \
    X(27,  raw_imu) \
    X(29,  scaled_pressure) \
    X(36,  servo_output_raw) \
    X(65,  RC_channels) \
    X(116, scaled_imu2)


#endif /* INC_FC_LOG_LOG_H_ */
