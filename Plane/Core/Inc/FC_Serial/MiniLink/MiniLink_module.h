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
#include <FC_Serial/MiniLink/MiniLink.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


/* Variables -----------------------------------------------------------------*/
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
		uint8_t nack : 1;
	} flag;
} MiniLinkPacket;


typedef struct{
	uint8_t* start;
	uint8_t* offset;
	uint16_t length;
} JumboPakcet;

extern JumboPakcet jumboTx;


/* Macros --------------------------------------------------------------------*/
#define LOG_MAVLINK_HEADER 0xFA
#define LOG_BUFFER_SIZE 1024

#define LOG_TABLE(X) \
    X(26,  scaled_imu) \
    X(27,  raw_imu) \
    X(29,  scaled_pressure) \
    X(30,  attitude) \
    X(36,  servo_output_raw) \
    X(65,  RC_channels) \
    X(116, scaled_imu2)	\
    X(129, scaled_imu3)	\


/* Functions -----------------------------------------------------------------*/
int MiniLink_Send();


/* Functions 1 ---------------------------------------------------------------*/
int Log_pack(uint16_t msgId, uint8_t* payload, uint8_t len);
int Log_transmit_UART(uint8_t *packet, uint8_t len);
int Log_addMailBox_CDC(uint8_t* packet, uint8_t len);
int Log_transmit_CDC();


/*
 * cal crc는 SRXL2.c에 존재
 * 추후 RadioControl.c로 이전
 * SRXL2.c 코드 정리
 * 	- readByte 내에 cal crc 수행 후 타입에 따라 리턴
 */
extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);


#endif /* INC_GCS_MAVLINK_GCS_COMMON_H_ */
