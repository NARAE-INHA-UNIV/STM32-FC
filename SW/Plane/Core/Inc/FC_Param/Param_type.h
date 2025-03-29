/*
 * Param_type.h
 *
 *  Created on: Mar 29, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_PARAM_PARAM_TYPE_H_
#define INC_FC_PARAM_PARAM_TYPE_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Param/Param.h>


/* Typedef -------------------------------------------------------------------*/

/* SERVO ---------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint8_t AUTO_TRIM : 1;
	uint16_t RATE;
	uint8_t DSHOT_RATE : 3;
	uint8_t DSHOT_ESC : 3;
	uint32_t GPIO_MASK;
	uint16_t RC_FS_MSK;
	uint8_t _32_ENABLE :1;
} PARAM_SERVO;

typedef struct __attribute__((packed)){
	uint16_t MIN;
	uint16_t MAX;
	uint16_t TRIM;
	uint8_t REVERSED : 1;
	int16_t FUNCTION;
} PARAM_SERVO_CH;


/* RC ------------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	float OVERRIDE_TIME;
	uint16_t OPTIONS;
	uint16_t PROTOCOLS;
	float FS_TIMEOUT;
	uint32_t reversedMask;
} PARAM_RC;

typedef struct __attribute__((packed)){
	uint16_t MIN;
	uint16_t MAX;
	uint16_t TRIM;
	uint8_t DZ;
	uint16_t OPTION;
} PARAM_RC_CH;

typedef struct __attribute__((packed)){
	uint8_t THR;
	uint8_t ROL;
	uint8_t PIT;
	uint8_t YAW;
} PARAM_RC_MAP;


#endif /* INC_FC_PARAM_PARAM_TYPE_H_ */
