/*
 * Log.c
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Log/Log.h>


/* Functions -----------------------------------------------------------------*/
int Log_Send()
{
    const Log_Header header = {
        0x5305,
        sizeof(Log_Header)
        +sizeof(uint8_t)+sizeof(uint16_t)+sizeof(RC_Channel)
        +sizeof(uint8_t)+sizeof(uint16_t)+sizeof(GYRO_ICM42688)
    };

	while(1 == CDC_Transmit_FS(&header, sizeof(Log_Header))) {}

	/* LOG DATA */
	Log_transmit((uint8_t*)&RC_channel, sizeof(RC_Channel));
	Log_transmit((uint8_t*)&ICM42688, sizeof(GYRO_ICM42688));

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

	while(1 == CDC_Transmit_FS(&len, sizeof(uint8_t))) {}
	while(1 == CDC_Transmit_FS(p, len)) {}
	while(1 == CDC_Transmit_FS((uint8_t*)&crc, sizeof(uint16_t))) {}
	return len;
}
