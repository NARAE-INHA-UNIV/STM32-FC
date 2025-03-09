/*
 * FC_Basic/Buzzer.h
 *
 *  Created on: Feb 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#include <FC_Basic/Buzzer.h>
#include "main.h"


void BuzzerPlayNote(Note note){
	TIM4->ARR = APB1_CLOCKS/TIM4->PSC/tones[note];
	return;
}

void BuzzerPlayInit(void){
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

void BuzzerPlayOneCycle(void){
	LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	TIM4->PSC = 5;
	TIM4->CCR3 = TIM4->ARR/2;

	for (int i=0; i<8; i++){
		BuzzerPlayNote(i);
		HAL_Delay(100);
	}
	LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
	return;
}

