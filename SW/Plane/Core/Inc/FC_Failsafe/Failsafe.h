/*
 * Failsafe.h
 *
 *  Created on: Mar 29, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_FAILSAFE_FAILSAFE_H_
#define INC_FC_FAILSAFE_FAILSAFE_H_


#include <main.h>
#include <FC_RC/driver_RC.h>
#include <FC_Servo/driver_Servo.h>

extern uint8_t fsFlag;

void FS_mannualMode(void);

#endif /* INC_FC_FAILSAFE_FAILSAFE_H_ */
