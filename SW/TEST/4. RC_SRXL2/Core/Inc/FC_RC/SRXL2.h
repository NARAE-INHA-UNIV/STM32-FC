/*
 * FC_RC/SRXL2.h
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_SRXL2_H_
#define INC_FC_RC_SRXL2_H_

#include <FC_RC/spm_srxl.h>
#include <FC_RC/driver_SRXL2.h>
#include <FC_Basic/RingBuffer.h>
#include <main.h>

#define SRXL2_RING_BUFFER_SIZE 128

RingFifo_t SRXL2_RingFifo;

/* 0bnm
 * n : 수신 후 4ms  대기 플래그
 * m : 수신 후 확인 플래그
 */
uint8_t SRXL2_flag;
uint8_t SRXL2_data[SRXL_MAX_BUFFER_SIZE];


// INDEX
#define SRXL2_INDEX_PACKET_TYPE 1
#define SRXL2_INDEX_LENGTH 2


typedef struct {
	uint8_t SRXL2_ID;
	uint8_t PacketType;
	uint8_t Length;
	uint8_t *Data;
	uint16_t crc;
} SRXL2_Packet;

SRXL2_Packet packet;

typedef struct {
	uint8_t SrcID;
	uint8_t DestID;
	uint8_t Priority;
	uint8_t BaudRate;
	uint8_t Info;
	uint32_t UID;
} SRXL2_Handshake_Data;

SRXL2_Handshake_Data handshakeRx;



uint16_t calculate_crc(uint8_t *data, uint8_t length);
int SRXL2_readByte(void);
int SRXL2_doHandshake(void);
int SRXL2_doBind(void);
int SRXL2_Transmit(uint8_t *data, uint8_t len);

#endif /* INC_RC_SRXL2_H_ */
