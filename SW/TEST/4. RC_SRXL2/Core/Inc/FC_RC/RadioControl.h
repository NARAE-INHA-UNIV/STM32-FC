/*
 * FC_RC/RadioControl.h
 * Radio 범용 라이브러리
 *
 *  Created on: Mar 10, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_RADIOCONTROL_H_
#define INC_FC_RC_RADIOCONTROL_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <FC_Basic/RingBuffer.h>


/* Macro ---------------------------------------------------------------------*/
#define RC_CHANNEL_MAX (32)


/* Variables -----------------------------------------------------------------*/
extern RingFifo_t RC_rxRingFifo;

typedef struct {
	uint8_t half_tx : 1;	/** 0 : 송신 아님, 1 : 송신 맞음 **/
	uint8_t half_using : 1;	/** 0 : 송신 가능, 1 : 송신 불가 **/
	uint8_t uart : 1;		/** 0 : 수신 없음, 1 : 수신 있음 **/
} RC_Receive_Flag;

extern RC_Receive_Flag RC_rxFlag;


typedef struct __attribute__((packed)){
	uint16_t PROTOCOLS;
	float FS_TIMEOUT;
	uint32_t reversedMask;

	struct __attribute__((packed)){
		uint16_t MIN;
		uint16_t MAX;
		uint16_t TRIM;
		uint8_t DZ;
		uint16_t OPTION;
	} CHANNEL[RC_CHANNEL_MAX];

	struct __attribute__((packed)){
		uint8_t THR;
		uint8_t ROL;
		uint8_t PIT;
		uint8_t YAW;
	} MAP;
} PARM_RC;

extern PARM_RC PARM_rc;


typedef struct __attribute__((packed)){
	uint8_t rssi;
	uint32_t mask;
	uint16_t value[RC_CHANNEL_MAX];
} RC_Channel;

extern RC_Channel RC_channel;


/* Functions -----------------------------------------------------------------*/
int RC_halfDuplex_Transmit(uint8_t *data, uint8_t len);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#endif /* INC_FC_RC_RADIOCONTROL_H_ */
