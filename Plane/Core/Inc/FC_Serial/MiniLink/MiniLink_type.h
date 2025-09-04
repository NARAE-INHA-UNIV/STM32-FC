/*
 * MiniLink_type.h
 *
 *  Created on: Sep 4, 2025
 *      Author: leecurrent04@inha.edu
 */

#ifndef INC_FC_SERIAL_MINILINK_MINILINK_TYPE_H_
#define INC_FC_SERIAL_MINILINK_MINILINK_TYPE_H_


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


// MAVLink 패킷 구조의 인덱스를 정의
typedef enum {
    MAVLINK_PACKET_INDEX_STX = 0,
    MAVLINK_PACKET_INDEX_LENGTH,
    MAVLINK_PACKET_INDEX_SEQ,
    MAVLINK_PACKET_INDEX_MSGID_L,
    MAVLINK_PACKET_INDEX_MSGID_H ,
    MAVLINK_PACKET_INDEX_PAYLOAD,
} MAVLink_PacketIndex;



typedef struct{
	uint8_t* start;
	uint8_t* offset;
	uint16_t length;
} JumboPakcet;


#endif /* INC_FC_SERIAL_MINILINK_MINILINK_TYPE_H_ */
