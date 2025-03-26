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
/*
 * SYSTEM_TIME (2)
 * The system time is the time of the master clock, typically the computer clock of the main onboard computer.
 */
typedef struct __attribute__((packed)){
	uint64_t time_unix_usec;	// Timestamp (UNIX epoch time). (us)
	uint32_t time_boot_ms;		// Timestamp (time since system boot). (ms)
} SYSTEM_TIME;

extern SYSTEM_TIME system_time;

/* Functions -----------------------------------------------------------------*/
int Log_Send();
int Log_transmit(uint8_t* p, uint8_t len);


#endif /* INC_FC_LOG_LOG_H_ */
