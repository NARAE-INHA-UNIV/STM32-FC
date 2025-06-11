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

#include <FC_Param/Param_type.h>

#include <FC_RC/RadioControl.h>
#include <FC_Servo/Servo.h>


/* Variables -----------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	PARAM_HEADER header;
	PARAM_SERVO servo;
	PARAM_RC rc;
} PARAM;

extern PARAM param;

/* Functions -----------------------------------------------------------------*/

#endif /* INC_FC_PARAM_PARAM_H_ */
