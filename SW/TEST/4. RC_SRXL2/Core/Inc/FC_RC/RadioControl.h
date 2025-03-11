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


/* Variables -----------------------------------------------------------------*/
extern RingFifo_t RC_rxRingFifo;

typedef struct {
	uint8_t half_tx : 1;	/** 0 : 송신 아님, 1 : 송신 맞음 **/
	uint8_t half_using : 1;	/** 0 : 송신 가능, 1 : 송신 불가 **/
	uint8_t uart : 1;		/** 0 : 수신 없음, 1 : 수신 있음 **/
} RC_Receive_Flag;

extern RC_Receive_Flag RC_rxFlag;
extern uint16_t* RC_Channel;
extern uint8_t RC_ChannelNum;


/* Functions -----------------------------------------------------------------*/
int RC_halfDuplex_Transmit(uint8_t *data, uint8_t len);

#endif /* INC_FC_RC_RADIOCONTROL_H_ */
