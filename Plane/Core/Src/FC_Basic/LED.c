/*
 * LED.c
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 *
 *      자세한 것은 STM32-FC-doc 참고..
 *      shit..
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Basic/LED/LED_module.h>
#include <main.h>

#include <FC_Serial/MiniLink/MiniLink.h>


/* Variables -----------------------------------------------------------------*/
LED_CONTROL control[3];


/* Functions -----------------------------------------------------------------*/
void LED_Update(void)
{
	for(uint8_t i=0; i<sizeof(control)/sizeof(control[0]); i++)
	{
		if(msg.system_time.time_boot_ms - control[i].previous_change_time < control[i].next_change_time) continue;
		control[i].previous_change_time = msg.system_time.time_boot_ms;

		if(control[i].enabled == 0 ) continue;

		uint8_t enable = GET_BIT(control[i].blink_pattern, control[i].shifter++);
		control[i].next_change_time = (enable==1?2000:500);

		if(enable){
			if(control[i].previous_long_bit)
			{
				control[i].shifter = 0;
				control[i].next_change_time = 0;
				control[i].previous_long_bit = 0;
				continue;
			}
			control[i].previous_long_bit = 1;
		}
		else control[i].previous_long_bit = 0;


		if(control[i].led_state){
			LED_controlOFF(i);
		}
		else{
			LED_controlON(i);
		}
	}
}


void LED_SetRed(uint8_t state)
{
	if(state == 0)
	{
		control[0].enabled = 0;
		return;
	}

	control[0].enabled = 1;
	control[0].blink_pattern = LED_makeBlinkPattern(state);

	return;
}


void LED_SetYellow(uint8_t state)
{
	if(state == 0)
	{
		control[1].enabled = 0;
		return;
	}

	control[1].enabled = 1;
	control[1].blink_pattern = LED_makeBlinkPattern(state);
	return;
}


void LED_SetBlue(uint8_t state)
{
	if(state == 0)
	{
		control[2].enabled = 0;
		return;
	}

	control[2].enabled = 1;
	control[2].blink_pattern = LED_makeBlinkPattern(state);
	return;
}

void LED_ResetRed(void)
{
	control[0].enabled = 0;
	return;
}


void LED_ResetYello(void)
{
	control[1].enabled = 0;
	return;
}


void LED_ResetBlue(void)
{
	control[2].enabled = 0;
	return;
}


/* Functions 1 ---------------------------------------------------------------*/
void LED_controlON(uint8_t index)
{
	control[index].led_state = 1;

	switch(index)
	{
	case 0: LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin); break;
	case 1: LL_GPIO_SetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin); break;
	case 2: LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin); break;
	}
	return;
}

void LED_controlOFF(uint8_t index)
{
	control[index].led_state = 0;

	switch(index)
	{
	case 0: LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin); break;
	case 1: LL_GPIO_ResetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin); break;
	case 2: LL_GPIO_ResetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin); break;
	}
	return;
}

/*
 * @brirf 상태 값을 입력 받으면 Blink Pattern을 만드는 함수
 * @detail
 * 		state 값이 uint8_t 이므로 최대 8개의 장치 오류 표현 가능
 * 		최상위 비트는 마지막임을 나타내는 것
 * 		깜빡-깜빡깜빡깜빡-
 *
 * 		빡 : 0, 빡- : 1
 * 		ex) 5 (0b101) : 0b110000010
 * @param state
 * @retval blink_pattern
 */
uint64_t LED_makeBlinkPattern(uint8_t state)
{
	uint16_t pattern = 0;

	uint8_t sum = 0;
	for(uint8_t i=0; i<sizeof(uint8_t)*8; i++)
	{
		if(GET_BIT(state, i) == 0) continue;
		sum += 2*(i+1);
		pattern |= 0x1<<(sum-1);
	}
	pattern |= 0x1<<(sum);

	return pattern;
}
