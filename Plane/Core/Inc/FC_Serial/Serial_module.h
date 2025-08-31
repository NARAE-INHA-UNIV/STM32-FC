/*
 * Serial_module.h
 *
 *  Created on: Jul 13, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERIAL_SERIAL_MODULE_H_
#define INC_FC_SERIAL_SERIAL_MODULE_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/Serial.h>

#include <stddef.h>

#include <FC_Serial/MiniLink/MiniLink.h>
#include <FC_Serial/MiniLink/MiniLink_module.h>


/* Variables -----------------------------------------------------------------*/
extern MiniLinkPacket serialRX;


/* Functions -----------------------------------------------------------------*/


/* Functions 3 ---------------------------------------------------------------*/
void SERIAL_receivedIRQ2(uint8_t serialNumber, uint8_t data);
void SERIAL_receviedParser(uint8_t* Buf, uint32_t Len);

extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);


#endif /* INC_FC_SERIAL_SERIAL_MODULE_H_ */
