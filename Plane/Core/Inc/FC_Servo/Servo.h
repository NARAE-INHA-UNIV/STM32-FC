/*
 * driver.h
 *
 *  Created on: Mar 27, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERVO_SERVO_H_
#define INC_FC_SERVO_SERVO_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
int SERVO_Initialization(void);

void SERVO_doArm(void);
void SERVO_doDisarm(void);
void SERVO_control(void);

void SERVO_setFailsafe(void);
void SERVO_doCalibrate(uint8_t mode);


#endif /* INC_FC_SERVO_DRIVER_SERVO_H_ */
