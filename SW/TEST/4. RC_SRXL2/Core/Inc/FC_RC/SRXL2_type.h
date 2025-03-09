/*
 * FC_RC/SRXL2_type.h
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_SRXL2_TYPE_H_
#define INC_FC_RC_SRXL2_TYPE_H_

#include "main.h"

typedef struct __attribute__((packed)){
	uint8_t speckrum_id;
	uint8_t pType;
	uint8_t len;
} SRXL2_Header;

typedef struct __attribute__((packed)){
	SRXL2_Header header;
	uint8_t *Data;
	uint16_t crc;
} SRXL2_Packet;


//      7.2 Handshake Packet
typedef struct __attribute__((packed)){
	uint8_t SrcID;
	uint8_t DestID;
	uint8_t Priority;
	uint8_t BaudRate;
	uint8_t Info;
	uint32_t UID;
} SRXL2_Handshake_Data;

typedef struct __attribute__((packed)){
	SRXL2_Header header;
	SRXL2_Handshake_Data data;
	uint16_t crc;
}SRXL2_Handshake_Packet;


//      7.3 Bind Info Packet


//      7.7 Control Data Packet
typedef struct {
	uint8_t Command;
	uint8_t ReplyID;
	int8_t rssi;
	uint16_t frameLosses;
	uint32_t channlMask;
} SRXL2_Control_Data;


extern SRXL2_Packet packet;
// extern SRXL2_Handshake_Data rx_handshake_data;

#endif /* INC_FC_RC_SRXL2_TYPE_H_ */
