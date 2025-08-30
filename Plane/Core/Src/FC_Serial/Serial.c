/*
 * Serial.c
 *
 *  Created on: Jul 13, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 *
 *  @detail :
 *  	UART, CDC를 통합 관리하는 코드
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Basic/LED/LED.h>
#include <FC_Serial/Serial_module.h>


/* Variables -----------------------------------------------------------------*/
MiniLinkPacket serialRX;


/* Functions -----------------------------------------------------------------*/
int SERIAL_Initialization()
{
	// interrupt when finished receiving
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_RXNE(USART2);
	LL_USART_EnableIT_RXNE(USART3);
	LL_USART_EnableIT_RXNE(UART4);
	LL_USART_EnableIT_RXNE(UART5);

	jumboTx.start = (uint8_t*)malloc(sizeof(uint8_t)*LOG_BUFFER_SIZE);
	if(jumboTx.start == 0) { return 1; }
	jumboTx.offset = jumboTx.start;

	return 0;
}

int SERIAL_Handler()
{
	if(serialRX.flag.ack == 0)
	{
		MiniLink_Send();
		return 0;
	}

	switch(serialRX.header.msgId )
	{
	case 1:
		LED_SetRed(2);
		break;
	case 2:
		LED_SetYellow(serialRX.payload[0]);
		break;
	case 3:
		LED_SetBlue(serialRX.payload[0]);
		break;
	}

	serialRX.flag.ack = 0;
	free(serialRX.payload);

	return 0;
}

extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);
void SERIAL_receivedIRQ2(uint8_t serialNumber, uint8_t data)
{
	if(2 != param.serial[serialNumber].protocol)
	{
		return;
	}

	static uint8_t cnt = 0;
	static uint8_t* p;

	switch(cnt++)
	{
	case 0:
		if(LOG_MAVLINK_HEADER != data) cnt = 0;
		break;
	case 1:
		p = (uint8_t*)malloc(sizeof(uint8_t)*data);
		p[0] = LOG_MAVLINK_HEADER;
		p[1] = data;
		break;
	default :
		p[cnt-1] = data;

		if(cnt>=p[1])
		{
			uint8_t len = p[1];
			uint16_t crc = (uint16_t)p[len-2] << 8 | p[len-1];

			if(crc != calculate_crc(p, len)){
				free(p);
				cnt = 0;
				return;
			}

			if(serialRX.payload != 0){
				free(serialRX.payload);
			}
			serialRX.header.length = p[1]-7;
			serialRX.header.seq = p[2];
			serialRX.header.msgId = (uint16_t)p[3] << 8 | p[4];
			serialRX.payload = (uint8_t*)malloc((serialRX.header.length)*sizeof(uint8_t));
			memcpy(serialRX.payload, &p[5], serialRX.header.length);

			free(p);
			cnt = 0;
		}
		break;
	}

	return;
}



/* USB Functions -------------------------------------------------------------*/
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	if(Len<3 || Len > 255) return;
	if(Buf[0] != LOG_MAVLINK_HEADER) return;

	uint16_t crc = ((uint16_t)Buf[Len -2] << 8 | Buf[Len -1]);

	if(crc != calculate_crc(&Buf[0], (uint8_t)Len)){
		// NACK
		return;
	}

	if(serialRX.payload != 0){
		free(serialRX.payload);
	}


	serialRX.header.length = Buf[1]-7;
	serialRX.header.seq = Buf[2];
	serialRX.header.msgId = (uint16_t)Buf[4] << 8 | Buf[3];
	serialRX.payload = (uint8_t*)malloc((serialRX.header.length)*sizeof(uint8_t));
	memcpy(serialRX.payload, &Buf[5], serialRX.header.length);

	serialRX.flag.ack = 1;

	return;
}
