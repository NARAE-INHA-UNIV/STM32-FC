/*
 * Failsafe.c
 *
 *  Created on: Mar 29, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


#include <AP_Failsafe/Failsafe_module.h>


uint8_t fsFlag = 0;


/* Functions -----------------------------------------------------------------*/
void FS_mannualMode(void)
{
	LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	// RTH가 있는지 확인

	// RTH가 없으면 쓰로틀 끄고 서보 중립
	// 추후 원주 비행 구현
	SERVO_setFailsafe();

	return;
}


int FS_IsFailsafe(void)
{
	if(fsFlag == 1){
		FS_mannualMode();
	}
	else{
//		LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
	}

	return 0;
}
