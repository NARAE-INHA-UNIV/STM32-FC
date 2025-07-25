/*
 * Serial.h
 *
 *  Created on: Jul 13, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERIAL_SERIAL_H_
#define INC_FC_SERIAL_SERIAL_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/driver.h>

#include <FC_Serial/MiniLink/driver.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
extern MiniLinkPacket serialRX;


/* Functions -----------------------------------------------------------------*/


/* Functions 3 ---------------------------------------------------------------*/
void SERIAL_receivedIRQ2(uint8_t serialNumber, uint8_t data);


#endif /* INC_FC_SERIAL_SERIAL_H_ */
