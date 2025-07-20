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

LogPacket logTx;


/* Functions -----------------------------------------------------------------*/
int Log_Send()
{
	static uint32_t previous_time = 0;

	// 10Hz 단위로 전송
	if(!(msg.system_time.time_boot_ms - previous_time > 100)) return -1;
	previous_time = msg.system_time.time_boot_ms;

	#define GENERATE_CASE(type, field) Log_transmit(type, (uint8_t*)&(msg.field), sizeof(msg.field));
	LOG_TABLE(GENERATE_CASE);
	#undef GENERATE_CASE
	return 0;
}


/*
 * cal crc는 SRXL2.c에 존재
 * 추후 RadioControl.c로 이전
 * SRXL2.c 코드 정리
 * 	- readByte 내에 cal crc 수행 후 타입에 따라 리턴
 */
extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);

/*
 * @brief 로그 전송
 * @detail 헤더 및 패킷의 길이 계산, CRC 입력
 * @parm *p : msg 하위 구조체(ex. msg.raw_imu)
 * 		 len : 구조체의 크기 (!= 로그 패킷의 길이가 아님)
 */
int Log_transmit(uint16_t msgId, uint8_t* p, uint8_t len)
{
	logTx.header = code;
	logTx.length = sizeof(LogPacket) + len + sizeof(uint16_t);
	logTx.seq++;
	logTx.msgId = msgId;

	// create array for packet
    uint8_t* packet = (uint8_t*)malloc(logTx.length);

    // insert header, length, seq, msg id, payload
    memcpy(packet, &logTx, sizeof(LogPacket));
    memcpy(packet + sizeof(LogPacket), p, len);

    // insert crc
    uint16_t crc = calculate_crc(packet, logTx.length);
    memcpy(packet + logTx.length - sizeof(uint16_t), &crc, sizeof(uint16_t));

    // 전송
	CDC_Transmit_FS(packet, logTx.length);

	for(int i=0; i<logTx.length; i++)
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

	return logTx.length;
}

