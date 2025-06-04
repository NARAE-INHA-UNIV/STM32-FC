/*
 * PPM.c
 *
 *  Created on: Apr 5, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_RC/Protocol/PPM.h>


/* Variables -----------------------------------------------------------------*/
/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief PPM 입력을 위한 타이머 설정
 * @detail RC_Initialization()에서 실행됨
 * @retval 0 : 정상 수신
 */
int PPM_init(void)
{
	/*
	 * TIM1은 168MHz이고 PPM은 일반적으로 20ms 이므로
	 */
	const int hz = 50;
	TIM1->ARR = 1000000/hz-1;
	TIM1->PSC = 168-1;
	TIM1->CCR4 = TIM1->ARR;

	LL_TIM_EnableCounter(TIM1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
	return 0;
}


/*
 * @brief PPM IRQ2
 * @detail RC_receiveIRQ2에서 호출됨
 * @parm timer 데이터 (ms 단위)
 * @retval 0 : 정상 수신
 * @retval 1 : 이상 데이터
 */
int PPM_readData(uint16_t data)
{
	static uint16_t previous = 0;
    static uint8_t cnt = 0;
	uint16_t rx = system_time.time_unix_usec - previous;

	previous = system_time.time_unix_usec;

    if(rx>2500) cnt = 0;
	if(rx>2200 || rx<800) return 1;

	((uint16_t*)RC_Buffer)[cnt] = rx;
    cnt++;

	return 0;
}


/*
 * @brief 조종 데이터 로딩
 * @detail RC_GetData()에서 실행됨
 * @retval 0 : 정상 수신
 * @retval -1 : 수신 버퍼 없음
 * @retval 0xf2 : FailSafe
 */
int PPM_getControlData(void)
{
	PARAM_RC_CH* paramCh = (PARAM_RC_CH*)&param.rc.channel[0];
	RC_CHANNELS* rc = &RC_channels;

	if(RC_isBufferInit() != 0) return -1;

	for(int i=0; i<PPM_MAX_CHANNEL; i++){
		// Reverse 처리
		uint16_t value = ((uint16_t*)RC_Buffer)[i];
		if((param.rc.reversedMask>>i)&0x01)
		{
			rc->value[i] = map(value,
					1000, 2000,
					paramCh[i].MAX, paramCh[i].MIN) + paramCh[i].TRIM;
		}
		else{
			rc->value[i] = map(value,
					1000, 2000,
					paramCh[i].MIN, paramCh[i].MAX) + paramCh[i].TRIM;
		}

		// Dead-zone 처리
		if(rc->value[i]>(1500-paramCh[i].DZ) && rc->value[i]<(1500+paramCh[i].DZ)){
			rc->value[i] = 1500;
		}
	}

	return 0;
}
