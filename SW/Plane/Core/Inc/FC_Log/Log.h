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
#include <GCS_MAVLink/GCS_Common.h>


/* Variables -----------------------------------------------------------------*/
extern SYSTEM_TIME system_time;
extern SERVO_OUTPUT_RAW servo_output_raw;
extern RC_CHANNELS RC_channels;


/* Functions -----------------------------------------------------------------*/
int Log_Send();
int Log_transmit(uint8_t* p, uint8_t len);


#endif /* INC_FC_LOG_LOG_H_ */
