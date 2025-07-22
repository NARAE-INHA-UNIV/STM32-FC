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
#include <FC_Serial/Serial.h>


/* Functions -----------------------------------------------------------------*/
int SERIAL_Initialization()
{
	jumboTx.start = (uint8_t*)malloc(sizeof(uint8_t)*LOG_BUFFER_SIZE);
	if(jumboTx.start == 0) { return 1; }
	jumboTx.offset = jumboTx.start;

	return 0;
}

int SERIAL_Send()
{
	Log_Send();

	return 0;
}

extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);
void SERIAL_receivedIRQ2(uint8_t serialNumber, uint8_t data)
{
	if(2 == param.serial[serialNumber].protocol)
	{
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
				if(crc == calculate_crc(p, len)){
//					logType = p[2];
				}

				free(p);
				cnt = 0;
			}
			break;
		}

	}
	return;
}



/* USB Functions -------------------------------------------------------------*/
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	if(Len<3 || Len > 255) return;
	if(Buf[0] != LOG_MAVLINK_HEADER) return;

	uint16_t crc = ((uint16_t)Buf[Len -2] << 8 | Buf[Len -1]);
	if(crc != calculate_crc(&Buf[0], (uint8_t)Len)) return;

//	logType = Buf[1];

	return;
}
