/*
 * SRXL_testing.c
 * 우선 순위가 낮으면서 테스트 중인 코드
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#include <FC_RC/SRXL2.h>

/*
 * (@ In Progress)
 * Control 패킷에서 ReplyID가 0x00 = 아무 장치도 응답 요구를 안함.
 * HandShake에서 장치 등록을 해줘야하는 것으로 추정
 *  0 : 전송 성공
 * -1 : 전송 실패
 */
int SRXL2_SendTelemetryData(void)
{
    uint8_t telm_packet[22] =
    {
		SPEKTRUM_SRXL_ID,
		SRXL_TELEM_ID,
		22,
        0x30,			// DeviceID (Receiver)
		0x50,			// source id
		0x30,			// secondary id
        0x00, 0x00,		// int16 field1
        0x00, 0x00,		// int16 field2
        0x00, 0x00,		// int16 field3
		0xB0, 0x00,		// uint16 field1
		0xB0, 0x00,		// uint16 field2
		0xB0, 0x00,		// uint16 field3
		0xB0, 0x00,		// uint16 field4
        0x00, 0x00   // CRC 자리 (계산 후 입력)
    };
    insert_crc(telm_packet, sizeof(telm_packet));

	return SRXL2_Transmit(telm_packet, sizeof(telm_packet));
}
