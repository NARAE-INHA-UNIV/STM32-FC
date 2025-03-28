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
#include <stdlib.h>

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
	RC_Buffer = malloc(SRXL_MAX_BUFFER_SIZE*sizeof(uint8_t));

	SRXL2_Connect();
	return 0;
}

int RC_GetData(void)
{
	SRXL2_GetData();
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
int RC_reviceIRQ2(const uint8_t data)
{
	if(RC_rxFlag.half_tx == 1) return 1;

	if(SRXL2_readByteIRQ2(data) == 0){
		RC_rxFlag.uart = 1;
		RC_rxFlag.half_using = 0;
	}

	return 0;
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
	LL_GPIO_SetOutputPin(GPS1_SW_LED_GPIO_Port, GPS1_SW_LED_Pin);
	if(RC_rxFlag.half_using == 1) return -1;

	RC_rxFlag.half_using = 1;
	RC_rxFlag.half_tx = 1;

	for(int i=0; i<len; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, data[i]);
	}
	LL_GPIO_ResetOutputPin(GPS1_SW_LED_GPIO_Port, GPS1_SW_LED_Pin);
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
