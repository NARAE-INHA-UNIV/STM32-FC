/*
 * MAVLink_module.h
 *
 *  Created on: Mar 27, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_GCS_MAVLINK_GCS_MAVLink_H_
#define INC_GCS_MAVLINK_GCS_MAVLink_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/MiniLink/MiniLink.h>
#include <FC_Serial/MiniLink/MiniLink_type.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


/* Variables -----------------------------------------------------------------*/
extern JumboPakcet jumboTx;


/* Macros --------------------------------------------------------------------*/
#define LOG_TABLE(X) \
    X(26,  scaled_imu) \
    X(27,  raw_imu) \
    X(29,  scaled_pressure) \
    X(30,  attitude) \
    X(36,  servo_output_raw) \
    X(65,  RC_channels) \
    X(116, scaled_imu2)	\
    X(129, scaled_imu3)	\

// Header size and Header+CRC size
#define MINILINK_HEADER_SIZE 		(sizeof(MiniLinkHeader))
#define MINILINK_MIN_PACKET_SIZE 	(sizeof(MiniLinkHeader)+sizeof(uint16_t))

#define MINILINK_STX_CODE 				(0xFA)
#define LOG_BUFFER_SIZE 			1024


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
