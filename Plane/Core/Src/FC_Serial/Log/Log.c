/*
 * Log.c
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/Log/Log.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include <FC_Param/Param.h>

/* Variables -----------------------------------------------------------------*/
const uint8_t code = 0xFD;
uint16_t logType = 26;

LogPacket logTx;
JumboPakcet jumboTx;


/* Functions -----------------------------------------------------------------*/
int Log_Send()
{
	static uint32_t previous_time = 0;

	// 10Hz 단위로 전송
	if(!(msg.system_time.time_boot_ms - previous_time > 100)) return -1;
	previous_time = msg.system_time.time_boot_ms;

#define GENERATE_CASE(type, field) Log_pack(type, (uint8_t*)&(msg.field), sizeof(msg.field));
	LOG_TABLE(GENERATE_CASE);
#undef GENERATE_CASE

	Log_transmit_CDC();
	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * cal crc는 SRXL2.c에 존재
 * 추후 RadioControl.c로 이전
 * SRXL2.c 코드 정리
 * 	- readByte 내에 cal crc 수행 후 타입에 따라 리턴
 */
extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);

/*
 * @brief log msg를 packet으로 묶음
 * @detail 헤더 및 패킷의 길이 계산, CRC 입력
 * @parm msgId : msg inedex
 * 			*p : msg struct (ex. msg.raw_imu)
 * 			len : sizeof(struct) (!not packet size)
 */
int Log_pack(uint16_t msgId, uint8_t* payload, uint8_t len)
{
	logTx.header = code;
	logTx.length = sizeof(LogPacket) + len + sizeof(uint16_t);
	logTx.seq++;
	logTx.msgId = msgId;

	// create array for packet
	uint8_t* packet = (uint8_t*)malloc(logTx.length);

	// insert header, length, seq, msg id, payload
	memcpy(packet, &logTx, sizeof(LogPacket));
	memcpy(packet + sizeof(LogPacket), payload, len);

	// insert crc
	uint16_t crc = calculate_crc(packet, logTx.length);
	memcpy(packet + logTx.length - sizeof(uint16_t), &crc, sizeof(uint16_t));

	Log_transmit_UART(packet, logTx.length);
	Log_addMailBox_CDC(packet, logTx.length);

	free(packet);
	return 0;
}

/*
 * @brief CDC 전송 버퍼 추가
 * @detail 버퍼에 추가하지 못하면 기존 버퍼를 출력 후 비움.
 * @parm *packet : packet
 * 		 len : packet size
 */
int Log_addMailBox_CDC(uint8_t* packet, uint8_t len)
{
	if(len > LOG_BUFFER_SIZE) { return -1; }
	if(jumboTx.start == 0) { return -2; }

	// check can insert packet to Jumbo packet
	jumboTx.length = (uint16_t)(jumboTx.offset-jumboTx.start);
	if(jumboTx.length+len >= LOG_BUFFER_SIZE)
	{
		// MailBox의 버퍼가 가득 찼으면 전달 후, 패킷 저장
		Log_transmit_CDC();

		Log_addMailBox_CDC(packet, len);

		return 0;
	}

	// insert packet
	memcpy(jumboTx.offset, packet, len);
	jumboTx.offset += len;

	return 0;

}

/*
 * @brief 로그 전송
 * @detail 헤더 및 패킷의 길이 계산, CRC 입력
 * @parm *p : msg 하위 구조체(ex. msg.raw_imu)
 * 		 len : 구조체의 크기 (!= 로그 패킷의 길이가 아님)
 */
int Log_transmit_UART(uint8_t* packet, uint8_t len)
{
	for(int i=0; i<len; i++)
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

	return 0;
}


int Log_transmit_CDC()
{
	jumboTx.length = (uint16_t)(jumboTx.offset-jumboTx.start);
	CDC_Transmit_FS(jumboTx.start, jumboTx.length);

	jumboTx.offset = jumboTx.start;

	return 0;
}
