/*
 * SRXL_testing.c
 * 우선 순위가 낮으면서 테스트 중인 코드
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#include <FC_RC/Protocol/SRXL2/SRXL2_module.h>


/*
 * (@ Work In Progress)
 * @brief 수신기와 Bind 동작 수행
 * @parm SRXL2_Bind_Packet * tx_packet
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 */
int SRXL2_doBind(SRXL2_Bind_Packet* tx_packet)
{
	uint8_t len = tx_packet->header.len;
	if(sizeof(*tx_packet) != len) return -2;

	insert_crc((uint8_t*)tx_packet, len);

	return RC_halfDuplex_Transmit((uint8_t*)tx_packet, len);
}


/*
 * (@ Work In Progress)
 * 장치간 Handshake 동작 수행
 * Bus내 연결된 장치 정보 알림
 *
 * @parm SRXL2_Handshake_Packet *packet
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 */
SRXL2_SignalQuality_Data SRXL2_reqSignalQuality()
{
	SRXL2_SignalQuality_Data *rx_data;
	SRXL2_SignalQuality_Data data;

	uint8_t tx_packet[10] ={
			SPEKTRUM_SRXL_ID,
			SRXL_RSSI_ID,
			0x0a,
			SRXL_RSSI_REQ_REQUEST,
			0x12, 0x34, 0x56, 0x78, 		// Antenna A,B,L,R
			0x00, 0x00						// CRC.
	};

	insert_crc(tx_packet, sizeof(tx_packet));

	while(RC_halfDuplex_Transmit(tx_packet, sizeof(tx_packet)));
	for(uint8_t i=0; i<10; i++)
	{

		if(SRXL2_isReceived()!=0) continue;
		// SRXL2_GetData();
		if(packet.header.pType == SRXL_RSSI_REQ_SEND)
		{
			rx_data = &((SRXL2_SignalQuality_Packet*)RC_Buffer)->data;
			data.Request = rx_data->Request;
			data.AntennaA = rx_data->AntennaA;
			data.AntennaB = rx_data->AntennaB;
			data.AntennaL = rx_data->AntennaL;
			data.AntennaR = rx_data->AntennaR;

			break;
		}

	}
	return data;
}


/*
 * (@ Work In Progress)
 * (@ In Progress)
 * 수신기에서 Control 패킷에 ReplyID를 전송함.
 * ReplyID가 Handshake에서 등록한 ID와 같다면 데이터 전송
 *  0 : 전송 성공
 * -1 : 전송 실패
 */
int SRXL2_SendTelemetryData(void)
{
	SRXL2_Control_Packet* rx_packet;
	rx_packet = (SRXL2_Control_Packet *)RC_Buffer;

	if(rx_packet->ReplyID != SRXL_FC_DEVICE_ID)
	{
		return -1;
	}


	// User Def 1
	uint8_t telm_packet[22] =
	{
		SPEKTRUM_SRXL_ID,
		SRXL_TELEM_ID,
		22,
		receiver_info.SrcID,	// DeviceID (Receiver)
		0x50,					// source id
		0x30,					// secondary id
		0x00, 0x00,				// int16 field1
		0x00, 0xa0,				// int16 field2
		0x00, 0x00,				// int16 field3
		0x00, 0xa0,				// uint16 field1
		0x00, 0x00,				// uint16 field2
		0x00, 0x00,				// uint16 field3
		0x00, 0x00,				// uint16 field4
		0x00, 0x00   			// CRC 자리 (계산 후 입력)
	};
	insert_crc(telm_packet, sizeof(telm_packet));

	return RC_halfDuplex_Transmit(telm_packet, sizeof(telm_packet));
}
