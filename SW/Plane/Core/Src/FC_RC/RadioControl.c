/*
 * RadioControl.c
 * Radio 범용 라이브러리
 *
 *  Created on: Mar 10, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 *
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_RC/RadioControl.h>


/* Variables -----------------------------------------------------------------*/
/*
 * @brief RC 수신 및 송신 플래그
 *
 * @parm uint8_t halft_tx	: Half-Duplex에서 송신임을 나타내는 플래그 (1bit)
 * @parm uint8_t half_using : Half-Duplex에서 중복을 막기 위한 타이머 플래그 (1bit)
 * @parm uint8_t uart 		: UART1 수신 인터럽트 (1bit)
 */
RC_Receive_Flag RC_rxFlag;
uint8_t* RC_Buffer = 0;


/* Functions -----------------------------------------------------------------*/
/*
 * @brief RC 초기 설정
 */
int RC_Initialization(void)
{
	LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

	for(int i=0; i<8*sizeof(paramRc.PROTOCOLS); i++)
	{
		if(!(paramRc.PROTOCOLS&(0x1<<i))) continue;

		switch(i){
		case PPM:
			RC_Buffer = malloc(PPM_MAX_BUFFER_SIZE*sizeof(uint16_t));
			PPM_init();
			break;
		case SRXL2:
			RC_Buffer = malloc(SRXL_MAX_BUFFER_SIZE*sizeof(uint8_t));
			SRXL2_connect();
			break;
		}

		/*
		 * Enable multiple receiver support
		 */
		if(paramRc.OPTIONS&(0x1<<10)) continue;
		else break;
	}

	while(RC_checkThrottle()){
		BuzzerEnableThrottleHigh();

		// ESC Calibration
		if(RC_enterESCcalibration()==0) break;
	}

	BuzzerDisableThrottleHigh();
	LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

	return 0;
}

/*
 * @brief 조종 데이터 로딩
 * @detail 프로토콜에 따라 다르게 동작
 * @retval 0 : 정상 수신
 * @retval -1 : 수신 버퍼 없음
 * @retval -2 : 조종 데이터가 아님
 * @retval 0xf2 : FailSafe
 */
int RC_GetData(void)
{
	int retVal = 0;

	for(int i=0; i<8*sizeof(paramRc.PROTOCOLS); i++)
	{
		if(!(paramRc.PROTOCOLS&(0x1<<i))) continue;

		switch(i){
		case PPM:
			retVal = PPM_getControlData();
			break;
		case SRXL2:
			retVal = SRXL2_getControlData();
			break;
		}

		/*
		 * Enable multiple receiver support
		 */
		if(paramRc.OPTIONS&(0x1<<10)) continue;
		else break;
	}

	if(retVal == -1 || retVal ==-2) return retVal;
	if(retVal!=0xf2) fsFlag = 0;

	return 0;
}

/*
 * @brief 수신 인터럽트 IRQ2
 *
 * @parm uint8_t data : packet 1byte
 * @retval 0 : IRQ2 처리 완료
 *
 * 모든 수신 패킷을 처리하면 RC_rxFlag를 1로 처리함.
 */
int RC_receiveIRQ2(const uint16_t data)
{

	for(int i=0; i<8*sizeof(paramRc.PROTOCOLS); i++)
	{
		if(!(paramRc.PROTOCOLS&(0x1<<i))) continue;

		switch(i){
		case PPM:
			PPM_readData(data);
			break;
		case SRXL2:
			// Half-Duplex에서 송신한 패킷을 무시
			if(RC_rxFlag.half_tx == 1) return 1;

			// 모든 바이트를 읽었는지 검사
			if(SRXL2_readByteIRQ2(data) == 0){
				RC_rxFlag.uart = 1;
				RC_rxFlag.half_using = 0;
			}
			break;
		}

		/*
		 * Enable multiple receiver support
		 */
		if(paramRc.OPTIONS&(0x1<<10)) continue;
		else break;
	}



	return 0;
}


/*
 * @brief Buffer가 설정 되었는지 확인
 *
 * @parm None
 * @retval 0 : 설정됨
 * @retval -1 : 설정되지 않음
 */
int RC_isBufferInit(void){
	if(RC_Buffer == 0) return -1;
	return 0;
}


/*
 * @brief 쓰로틀 체크
 *
 * @parm None
 * @retval 0 : 쓰로틀 정상
 * @retval -1 : 쓰로틀 비정상
 */
int RC_checkThrottle(void)
{
	while(RC_GetData()){}
	if(RC_channels.value[paramRcMap.THR]>1050) return -1;

	return 0;
}


/*
 * @brief ESC 캘리브레이션 진입
 * @detail 쓰로틀이 High인 상황이 5초 이상 지속될때 진입
 *
 * @parm None
 * @retval 1 : 5초가 지속되지 않았음.
 * @retval 0 : 캘리브레이션 수행됨
 */
int RC_enterESCcalibration()
{
	static uint32_t previous_time = 0;

	if(!(system_time.time_boot_ms - previous_time > 5000)) return 1;
	previous_time = system_time.time_boot_ms;
	BuzzerDisableThrottleHigh();

	while(1)
	{
		while(RC_GetData()){}
		if(RC_channels.value[paramRcMap.THR] > 1800){
			SERVO_doCalibrate(1);
			continue;
		}
		SERVO_doCalibrate(0);
		break;
	}
	return 0;
}

/*
 * @brief Failsafe 모드로 진입
 *
 * @retval 0 : Failsafe 해제됨
 */
int RC_setFailsafe(uint16_t protocol)
{
	if(paramRc.OPTIONS&(0x1<<10)){
		// 수신기 하나에서 FS 임을 알림
		return 0;
	}

	fsFlag = 1;

	return 0xf2;
}


/*
 * @brief RC 데이터 송신 (Half-Duplex)
 *
 * @parm uint8_t* data : data address
 * @parm uint8_t len : sizeof(data)
 * @retval 0 : 송신 완료.
 * @retval -1 : 송신 실패.
 */
int RC_halfDuplex_Transmit(uint8_t *data, uint8_t len)
{
	if(RC_rxFlag.half_using == 1) return -1;

	RC_rxFlag.half_using = 1;
	RC_rxFlag.half_tx = 1;

	for(int i=0; i<len; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, data[i]);
	}

	RC_rxFlag.half_tx = 0;
	return 0;
}


/*
 * @brief 입력 값의 범위를 바꾸는 mapping 함수
 * @parm uint16_t x : input
 * @parm uint16_t in_min : x의 최솟값
 * @parm uint16_t in_max : x의 최댓값
 * @parm uint16_t out_min : x의 최솟값
 * @parm uint16_t out_max : x의 최댓값
 * @retVal uint16_t : 변환 값
 */
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
