/*
 * SERVO.h
 *
 *  Created on: Mar 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERVO_SERVO_MODULE_H_
#define INC_FC_SERVO_SERVO_MODULE_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/MiniLink/MiniLink.h>
#include <FC_Servo/Servo.h>
#include <main.h>



/* Variables -----------------------------------------------------------------*/
extern const uint8_t SERVO_TIMER_MAP[];


/* Functions -----------------------------------------------------------------*/
int doArm2Channel(uint8_t ch, uint8_t state);
int doArm2Channels(uint8_t *pCh, uint8_t len, uint8_t state);
uint8_t configurePWM(uint16_t hz);

void calculateServoOutput(void);

void setPWM(void);
int setPWM2Channel(uint8_t ch, uint16_t value);
int setPWM2Channels(uint8_t *pCh, uint8_t len, uint16_t value);



#endif /* INC_FC_SERVO_SERVO_MODULE_H_ */
