/*
 * Development.h
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERIAL_MINILINK_MSG_CMD_DEVELOPMENT_H_
#define INC_FC_SERIAL_MINILINK_MSG_CMD_DEVELOPMENT_H_


#include <stdint.h>


/*
 * AIRSPEED (295) — [WIP]
 * WORK IN PROGRESS: Do not use in stable production environments (it may change).
 *
 * Airspeed information from a sensor.
 */
typedef struct __attribute__((packed)){
	uint8_t id;						// Sensor ID. Messages with same value are from the same source (instance).
	float airspeed;					// Calibrated airspeed (CAS).
	int16_t temperature;			// Temperature. INT16_MAX for value unknown/not supplied.
	float raw_press;				// Raw differential pressure. NaN for value unknown/not supplied.
	uint8_t flags;					// Airspeed sensor flags.
} AIRSPEED;


#endif /* INC_FC_SERIAL_MINILINK_MSG_CMD_DEVELOPMENT_H_ */
