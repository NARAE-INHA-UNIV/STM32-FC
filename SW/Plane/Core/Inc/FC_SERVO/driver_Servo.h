/*
 * driver_SERVO.h
 *
 *  Created on: Mar 27, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERVO_DRIVER_SERVO_H_
#define INC_FC_SERVO_DRIVER_SERVO_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Param/Param.h>


/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
void SERVO_Initialization(void);

void SERVO_doArm(void);
void SERVO_doDisarm(void);
void SERVO_control(void);

void SERVO_setFailsafe(void);


#endif /* INC_FC_SERVO_DRIVER_SERVO_H_ */
