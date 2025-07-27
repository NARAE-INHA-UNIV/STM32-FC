/*
 * LED.h
 *
 *  Created on: Jul 26, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_LED_LED_MODULE_H_
#define INC_FC_BASIC_LED_LED_MODULE_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_Basic/LED/LED.h>


/* Variables -----------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint64_t blink_pattern;				// LED가 켜고 꺼지는 시간을 정한 비트
	uint8_t shifter;					// timeMask의 bit shifter
	uint16_t next_change_time;			// delay time
	uint32_t previous_change_time;		// 이전 LED 상태 변화한 시각
	uint8_t previous_long_bit : 1;		// 이전 상황의 비트
	uint8_t led_state :1;				// 이전 LED의 상태
	uint8_t enabled : 1;				// 활성화/비활성화
} LED_CONTROL;


extern LED_CONTROL control[3];


/* Macros --------------------------------------------------------------------*/
#define GET_BIT(x,i) ((x>>i)&0x1)


/* Functions -----------------------------------------------------------------*/
void LED_controlON(uint8_t index);
void LED_controlOFF(uint8_t index);
uint64_t LED_makeBlinkPattern(uint8_t state);


#endif /* INC_FC_BASIC_LED_LED_MODULE_H_ */
