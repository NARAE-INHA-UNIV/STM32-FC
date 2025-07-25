/*
 * LED.c
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#include <main.h>
#include <FC_Serial/MiniLink/driver.h>

typedef struct __attribute__((packed)){
	uint64_t timeMask;
	uint8_t bit : 1;
	uint8_t state :1;
	uint8_t shifter : 1;
	uint32_t time : 1;
} LED_CONTROL;

LED_CONTROL control[3];


#define GET_BIT(x,i) (x>>i)&0x1
void LED_Update(void)
{
	for(uint8_t i=0; i<3; i++)
	{

		if(GET_BIT(control[i].timeMask, control[i].shifter++)){
			if(control[i].bit) control[i].shifter = 0;
			control[i].bit = 1;
		}
		else control[i].bit = 0;

		uint16_t delayTime =(control[i].bit==1?2000:500);
		if(control[i].shifter==0) delayTime = 0;
		if(msg.system_time.time_boot_ms - control[i].time >= delayTime)
		{
			control[i].time = msg.system_time.time_boot_ms;
			if(control[i].state){
				LL_GPIO_ResetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
				control[i].state = 0;
			}
			else{
				LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
				control[i].state = 1;
			}
		}

	}
}

void LED_SetRadioControlIMU(uint8_t state)
{
	control[0].timeMask = 0;

	uint8_t sum = 0;
	for(uint8_t i=0; i<sizeof(uint8_t)*8; i++)
	{
		if(((state >> i) & 0x1) == 0) continue;
		control[0].timeMask |= 0x1<<(2*(i+1) + sum);
		sum += 2*i+2;
	}

	return;
}

void LED_SetAHRS(uint8_t state)
{
	return;
}


