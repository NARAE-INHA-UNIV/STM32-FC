/*
 * Log.c
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Log/Log.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include <FC_Param/Param.h>

/* Variables -----------------------------------------------------------------*/
const uint8_t code = 0xFD;
uint16_t logType = 26;

/* Functions -----------------------------------------------------------------*/
int Log_Send()
{
	static uint32_t previous_time = 0;

	// 10Hz 단위로 전송
	if(!(msg.system_time.time_boot_ms - previous_time > 100)) return -1;
	previous_time = msg.system_time.time_boot_ms;

	switch(logType)
	{
	case 26: LOG_TRANSMIT(msg.scaled_imu); break;
	case 27: LOG_TRANSMIT(msg.raw_imu); break;
	case 29: LOG_TRANSMIT(msg.scaled_pressure); break;
	case 36: LOG_TRANSMIT(msg.servo_output_raw); break;
	case 65: LOG_TRANSMIT(msg.RC_channels); break;
	case 116 : LOG_TRANSMIT(msg.scaled_imu2); break;
	default: break;
	}
	return 0;
}


/*
 * cal crc는 SRXL2.c에 존재
 * 추후 RadioControl.c로 이전
 * SRXL2.c 코드 정리
 * 	- readByte 내에 cal crc 수행 후 타입에 따라 리턴
 */
extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);
int Log_transmit(uint8_t* p, uint8_t len)
{

    uint8_t packetLen = len+sizeof(uint8_t)*3;
    uint8_t* packet = malloc(packetLen);

    memcpy(packet, &code, sizeof(uint8_t));
    memcpy(packet + sizeof(uint8_t), p, len);

    uint16_t crc = calculate_crc(packet, packetLen);

    memcpy(packet + sizeof(uint8_t) + len, &crc, sizeof(uint16_t));

	CDC_Transmit_FS(packet, packetLen);

	for(int i=0; i<packetLen; i++)
	{
		if(param.serial[1].protocol == 2)
		{
			while(!LL_USART_IsActiveFlag_TXE(USART2)){}
			LL_USART_TransmitData8(USART2, packet[i]);
		}

		if(param.serial[2].protocol == 2)
		{
			while(!LL_USART_IsActiveFlag_TXE(USART3)){}
			LL_USART_TransmitData8(USART3, packet[i]);
		}
	}

    free(packet);

	return packetLen;
}

