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


/* Functions -----------------------------------------------------------------*/
int Log_Send()
{
	const uint8_t code = 0xFD;
	static uint32_t previous_time = 0;

	// 4Hz 단위로 전송
	if(!(system_time.time_boot_ms - previous_time > 250)) return -1;
	previous_time = system_time.time_boot_ms;

	CDC_Transmit_FS(&code, sizeof(code));

	Log_transmit((uint8_t *)&servo_output_raw, sizeof(servo_output_raw));
	// Log_transmit(&RC_channels, sizeof(RC_channels));
	// while(1 == CDC_Transmit_FS(&RC_channels, sizeof(RC_CHANNELS))) {}
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
	uint16_t crc = calculate_crc(p, len);

	CDC_Transmit_FS(p, len);
	CDC_Transmit_FS((uint8_t*)&crc, sizeof(uint16_t));
	return len;
}

