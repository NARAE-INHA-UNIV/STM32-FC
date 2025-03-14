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
RingFifo_t RC_rxRingFifo;

/*
 * @brief RC 수신 및 송신 플래그
 *
 * @parm uint8_t halft_tx	: Half-Duplex에서 송신임을 나타내는 플래그 (1bit)
 * @parm uint8_t half_using : Half-Duplex에서 중복을 막기 위한 타이머 플래그 (1bit)
 * @parm uint8_t uart 		: UART1 수신 인터럽트 (1bit)
 */
RC_Receive_Flag RC_rxFlag;

uint16_t RC_Channel[RC_CHANNEL_MAX];
uint32_t RC_ChannelMask;


/* Functions -----------------------------------------------------------------*/
/*
 * @brief RC 데이터 송신 (Half-Duplex)
 *
 * @parm uint8_t* data : data address
 * @parm uint8_t len : sizeof(data)
 * @retval 0 : 송신 완료.
 * @retval -1 : 송신 실패.
 *
 * +디버깅을 위해 PC1 (RSSI)에 GPIO 연결함
 */
int RC_halfDuplex_Transmit(uint8_t *data, uint8_t len)
{
	if(RC_rxFlag.half_using == 1) return -1;

	RC_rxFlag.half_using = 1;
	RC_rxFlag.half_tx = 1;

	LL_GPIO_SetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);

	for(int i=0; i<len; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, data[i]);

		// while(!LL_USART_IsActiveFlag_TC(USART1));
	}
	LL_GPIO_ResetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
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
