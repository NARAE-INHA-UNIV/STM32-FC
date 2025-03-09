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

#include <FC_RC/spm_srxl.h>
#include <FC_RC/SRXL2_type.h>
#include <FC_RC/driver_SRXL2.h>


/* Macro ---------------------------------------------------------------------*/
#define SPEKTRUM_SRXL_ID        (0xA6)
#define SRXL_MAX_BUFFER_SIZE    (80)
#define SRXL_MAX_DEVICES        (16)
#define SRXL2_RING_BUFFER_SIZE	(128)


/* Variables -----------------------------------------------------------------*/
extern RingFifo_t SRXL2_RingFifo;
extern uint8_t SRXL2_flag;
extern uint8_t SRXL2_data[SRXL_MAX_BUFFER_SIZE];

extern SRXL2_Packet packet;


/* Functions -----------------------------------------------------------------*/
int SRXL2_readByte(void);
// int	SRXL2_parseHandshakeData(SRXL2_Packet *rx);
int SRXL2_parseControlData(void);

int SRXL2_doHandshake(SRXL2_Handshake_Packet *packet);
int SRXL2_doBind(void);

int SRXL2_Transmit(uint8_t *data, uint8_t len);
uint16_t calculate_crc(const uint8_t *data, uint8_t len);
uint16_t insert_crc(uint8_t *data, uint8_t len);

#endif /* INC_RC_SRXL2_H_ */
