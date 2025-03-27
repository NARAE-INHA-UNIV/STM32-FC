/*
 * Param.h
 *
 *  Created on: Mar 27, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_PARAM_PARAM_H_
#define INC_FC_PARAM_PARAM_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>


/* Variables -----------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint8_t AUTO_TRIM : 1;
	uint16_t RATE;
	uint8_t DSHOT_RATE : 3;
	uint8_t DSHOT_ESC : 3;
	uint32_t GPIO_MASK;
	uint16_t RC_FS_MSK;
	uint8_t _32_ENABLE :1;
} SERVO;


typedef struct __attribute__((packed)){
	uint16_t MIN;
	uint16_t MAX;
	uint16_t TRIM;
	uint8_t REVERSED : 1;
	int16_t FUNCTION;
} SERVO_CH;

#define NUM_SERVO_CHANNELS 12


/* Functions -----------------------------------------------------------------*/

#endif /* INC_FC_PARAM_PARAM_H_ */
