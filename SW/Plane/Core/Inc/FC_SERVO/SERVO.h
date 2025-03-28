/*
 * SERVO.h
 *
 *  Created on: Mar 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERVO_SERVO_H_
#define INC_FC_SERVO_SERVO_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <FC_Servo/driver_Servo.h>

#include <GCS_MAVLink/GCS_Common.h>


/* Macro ---------------------------------------------------------------------*/
#define SERVO_CHANNEL_MAX (12)


/* Variables -----------------------------------------------------------------*/
extern SYSTEM_TIME system_time;
extern SERVO_OUTPUT_RAW servo_output_raw;
extern RC_CHANNELS RC_channels;

extern const uint8_t SERVO_TIMER_MAP[];


/* Functions -----------------------------------------------------------------*/
uint8_t configurePWM(uint16_t hz);
void calculateServoOutput(void);
void controlPWM(void);




#endif /* INC_FC_SERVO_SERVO_H_ */
