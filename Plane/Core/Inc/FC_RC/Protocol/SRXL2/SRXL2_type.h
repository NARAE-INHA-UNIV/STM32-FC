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

#define SRXL_MAX_CHANNEL 		(32)

// 컴파일러가 패딩하는 것을 방지
typedef struct __attribute__((packed)){
	uint8_t speckrum_id;
	uint8_t pType;
	uint8_t len;
} SRXL2_Header;



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
typedef struct __attribute__((packed)){
	uint8_t Request;
	uint8_t DeviceID;
	uint8_t Type;
	uint8_t option;
	uint64_t GUID;
	uint32_t UID;
} SRXL2_Bind_Data;

typedef struct __attribute__((packed)){
	SRXL2_Header header;
	SRXL2_Bind_Data data;
	uint16_t crc;
}SRXL2_Bind_Packet;


//      7.5 Signal Quality Packet
typedef struct __attribute__((packed)){
	uint8_t Request;
	int8_t AntennaA;
	int8_t AntennaB;
	int8_t AntennaL;
	int8_t AntennaR;
} SRXL2_SignalQuality_Data;

typedef struct __attribute__((packed)){
	SRXL2_Header header;
	SRXL2_SignalQuality_Data data;
	uint16_t crc;
} SRXL2_SignalQuality_Packet;


//      7.7 Control Data Packet
typedef struct __attribute__((packed)){
	int8_t rssi;
	uint16_t frameLosses;
	uint32_t mask;
	uint16_t values[SRXL_MAX_CHANNEL];
} SRXL2_Channel_Data;

typedef struct __attribute__((packed)){
	int8_t rssiMin;
	uint16_t holds;
	uint32_t mask;
	uint16_t values[SRXL_MAX_CHANNEL];
} SRXL2_Channel_Failsafe;

typedef struct __attribute__((packed)){
	uint8_t band;
	uint8_t channel;
	uint8_t pit;
	uint8_t power;
	uint32_t powerDec;
	uint8_t region;
} SRXL2_Channel_VTX;

typedef struct __attribute__((packed)){
	SRXL2_Header header;
	uint8_t Command;
	uint8_t ReplyID;
	union {
		SRXL2_Channel_Data data;
		SRXL2_Channel_Failsafe failsafe;
		SRXL2_Channel_VTX vtx;
	};
	uint16_t crc;
} SRXL2_Control_Packet;


// Common
typedef struct __attribute__((packed)){
	SRXL2_Header header;
	union {
		uint8_t *Data;
		SRXL2_Handshake_Data HandS;
		SRXL2_Bind_Data Bind;
		SRXL2_SignalQuality_Data SigQ;
	};
	uint16_t crc;
} SRXL2_Packet;


#endif /* INC_FC_RC_SRXL2_TYPE_H_ */
