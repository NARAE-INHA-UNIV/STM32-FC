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
#include <main.h>

#include <FC_Param/Param.h>
#include <FC_Serial/Log/Log.h>


/* Functions -----------------------------------------------------------------*/
int SERIAL_Initialization();
int SERIAL_Send();

/* Functions 3 ---------------------------------------------------------------*/
void SERIAL_receivedIRQ2(uint8_t serialNumber, uint8_t data);


#endif /* INC_FC_SERIAL_SERIAL_H_ */
