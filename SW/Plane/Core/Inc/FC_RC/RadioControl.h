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
#include <stdlib.h>

#include <FC_Param/Param.h>
#include <GCS_MAVLink/GCS_Common.h>

#include <FC_Basic/driver_Buzzer.h>
#include <FC_Servo/driver_Servo.h>
#include <FC_Failsafe/Failsafe.h>

#include <FC_RC/driver_RC.h>
#include <FC_RC/Protocol/SRXL2.h>


/* Macro ---------------------------------------------------------------------*/
#define RC_CHANNEL_MAX (18)


/* Variables -----------------------------------------------------------------*/
extern uint8_t* RC_Buffer;

typedef struct {
	uint8_t half_tx : 1;	/** 0 : 송신 아님, 1 : 송신 맞음 **/
	uint8_t half_using : 1;	/** 0 : 송신 가능, 1 : 송신 불가 **/
	uint8_t uart : 1;		/** 0 : 수신 없음, 1 : 수신 있음 **/
} RC_Receive_Flag;

extern RC_Receive_Flag RC_rxFlag;


enum RC_PARM_PROTOCOL{
	All 	 = 0,
	PPM 	 = 1,
	IBUS 	 = 2,
	SBUS 	 = 3,
	SBUS_NI  = 4,
	DSM 	 = 5,
	SUMD 	 = 6,
	SRXL 	 = 7,
	SRXL2 	 = 8,
	CRSF 	 = 9 ,
	ST24 	 = 10,
	FPORT	 = 11,
	FPORT2 	 = 12,
	FastSBUS = 13,
	DroneCAN = 14,
	Ghost	 = 15,
	MAVRadio = 16,
};



/* Functions -----------------------------------------------------------------*/
int RC_reviceIRQ2(const uint8_t data);
int RC_isBufferInit(void);

int RC_checkThrottle(void);
int RC_setFailsafe(uint16_t protocol);

int RC_halfDuplex_Transmit(uint8_t *data, uint8_t len);

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

#endif /* INC_FC_RC_RADIOCONTROL_H_ */
