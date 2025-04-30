/*
 * Log.c
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Log/Log.h>
#include <GCS_MAVLink/GCS_Common.h>


/* Variables -----------------------------------------------------------------*/
const uint8_t code = 0xFD;


/* Functions -----------------------------------------------------------------*/
int Log_Send()
{
	static uint32_t previous_time = 0;

	// 10Hz 단위로 전송
	if(!(system_time.time_boot_ms - previous_time > 100)) return -1;
	previous_time = system_time.time_boot_ms;

	Log_transmit((uint8_t*)&scaled_imu, sizeof(scaled_imu));
//	Log_transmit((uint8_t*)&raw_imu, sizeof(raw_imu));
//	Log_transmit((uint8_t *)&servo_output_raw, sizeof(servo_output_raw));
//	Log_transmit((uint8_t*)&RC_channels, sizeof(RC_channels));
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
		while(!LL_USART_IsActiveFlag_TXE(USART2)){}
		LL_USART_TransmitData8(USART2, packet[i]);
	}

    free(packet);

	return packetLen;
}


void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	if(Len<3 || Len > 255) return;
	if(Buf[0] != code) return;

	// calculate_crc(&Buf[0], (uint8_t)Len)

	return;
}
