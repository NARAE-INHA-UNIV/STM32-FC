/*
 * FC_Basic/Buzzer/driver.h
 *
 *  Created on: Mar 8, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_BUZZER_BUZZER_H_
#define INC_FC_BASIC_BUZZER_BUZZER_H_


/* Functions -----------------------------------------------------------------*/
void BuzzerPlayInit(void);
void BuzzerPlayOneCycle(void);


/* Functions (RC Alarm) ------------------------------------------------------*/
void BuzzerEnableThrottleHigh(void);
void BuzzerDisableThrottleHigh(void);


#endif /* INC_FC_BASIC_BUZZER_BUZZER_H_ */
