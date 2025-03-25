/*
 * Log.h
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_LOG_LOG_H_
#define INC_FC_LOG_LOG_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_RC/RadioControl.h>
#include <FC_Gyro/driver_ICM42688.h>

/* Variables -----------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint16_t id;
	uint16_t length;
} Log_Header;


/* Functions -----------------------------------------------------------------*/
int Log_Send();
int Log_transmit(uint8_t* p, uint8_t len);


#endif /* INC_FC_LOG_LOG_H_ */
