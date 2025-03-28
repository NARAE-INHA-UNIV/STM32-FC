/*
 * FC_RC/SRXL2.h
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_SRXL2_H_
#define INC_FC_RC_SRXL2_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <FC_Basic/RingBuffer.h>
#include <FC_RC/driver_RC.h>
#include <FC_RC/RadioControl.h>

#include <FC_RC/spm_srxl.h>
#include <FC_RC/SRXL2_type.h>


/* Macro ---------------------------------------------------------------------*/
#define SPEKTRUM_SRXL_ID        (0xA6)
#define SRXL_MAX_BUFFER_SIZE    (80)
#define SRXL_MAX_DEVICES        (16)
#define SRXL2_RING_BUFFER_SIZE	(128)


#define SRXL_CTRL_VALUE_MIN 	(10912)
#define SRXL_CTRL_VALUE_MAX 	(54612)


/* Variables -----------------------------------------------------------------*/
extern uint8_t* RC_Buffer;
extern SRXL2_Packet packet;
extern SRXL2_Handshake_Data receiver_info;
extern const uint8_t SRXL_FC_DEVICE_ID;


/* Functions -----------------------------------------------------------------*/
int SRXL2_Connect(void);
int SRXL2_GetData(void);

// int	SRXL2_parseHandshakeData(SRXL2_Packet *rx);
int SRXL2_parseControlData(SRXL2_Control_Packet *rx);

int SRXL2_doHandshake(SRXL2_Handshake_Packet *tx_packet);
int SRXL2_doBind(SRXL2_Bind_Packet* tx_packet);


/* Functions 2 ---------------------------------------------------------------*/
int SRXL2_isReceived(void);
int SRXL2_readByteIRQ2(const uint8_t data);


/* Functions 3 ---------------------------------------------------------------*/
uint16_t calculate_crc(const uint8_t *data, uint8_t len);
uint16_t insert_crc(uint8_t *data, uint8_t len);
uint8_t countSetBits(uint32_t i);


// In Progress!
SRXL2_SignalQuality_Data SRXL2_reqSignalQuality(void);
int SRXL2_SendTelemetryData(void);
#endif /* INC_RC_SRXL2_H_ */
