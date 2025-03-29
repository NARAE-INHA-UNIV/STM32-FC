/*
 * FC_Basic/Buzzer.h
 *
 *  Created on: Feb 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#include <FC_Basic/Buzzer.h>
#include "main.h"


void BuzzerPlayNote(Note note, uint16_t time)
{
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	TIM4->PSC = 4;
	TIM4->ARR = APB1_CLOCKS/TIM4->PSC/tones[note];
	TIM4->CCR3 = TIM4->ARR/2;

	HAL_Delay(time);
	LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	return;
}

void BuzzerPlayInit(void)
{
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	TIM4->ARR = 21;
	TIM4->CCR3 = TIM4->ARR/2;

	TIM4->PSC = 2000;
	HAL_Delay(100);
	TIM4->PSC = 1500;
	HAL_Delay(100);
	TIM4->PSC = 1000;
	HAL_Delay(100);

	LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	return;
}

void BuzzerPlayOneCycle(void)
{
	for (int i=0; i<8; i++){
		BuzzerPlayNote(i, 150);
	}
	return;
}


/* Functions (RC Alarm) ------------------------------------------------------*/
void BuzzerEnableThrottleHigh(void)
{
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	TIM4->ARR = 21;
	TIM4->CCR3 = TIM4->ARR/2;
	TIM4->PSC = 2000;

	return;
}

void BuzzerDisableThrottleHigh(void)
{
	LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	return;
}


int BuzzerToggleThrottleHigh(uint16_t delayTime)
{
	static uint32_t previous_time = 0;
	static uint8_t state = 0;
	if(!(system_time.time_boot_ms - previous_time > delayTime)) return -1;
	previous_time = system_time.time_boot_ms;

	if(state)
	{
		LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
		return 0;
	}
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	TIM4->ARR = 21;
	TIM4->CCR3 = TIM4->ARR/2;
	TIM4->PSC = 2000;

	return 1;
}
