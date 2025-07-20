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


/* Variables -----------------------------------------------------------------*/
extern const uint8_t code;
extern uint16_t logType;

typedef struct __attribute__((packed)){
	uint8_t header;
	uint8_t length;
	uint8_t seq;
	uint16_t msgId;
} LogPacket;

/* Functions -----------------------------------------------------------------*/
int Log_Send();
int Log_transmit(uint16_t msgId, uint8_t* p, uint8_t len);


/* Macros --------------------------------------------------------------------*/
#define LOG_TABLE(X) \
    X(26,  scaled_imu) \
    X(27,  raw_imu) \
    X(29,  scaled_pressure) \
    X(36,  servo_output_raw) \
    X(65,  RC_channels) \
    X(116, scaled_imu2)

#define LOG_TRANSMIT(a, x) Log_transmit(a, (uint8_t*)&(x), sizeof(x))


#endif /* INC_FC_LOG_LOG_H_ */
