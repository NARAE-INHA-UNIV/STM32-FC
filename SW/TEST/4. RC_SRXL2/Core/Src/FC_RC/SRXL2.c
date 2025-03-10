/*
 * FC_RC/SRXL2.c
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_RC/SRXL2.h>


/* Variables -----------------------------------------------------------------*/
uint8_t SRXL2_data[SRXL_MAX_BUFFER_SIZE];
SRXL2_Packet packet;
SRXL2_Handshake_Data receiver_info;


/* driver_SRXL2.h ------------------------------------------------------------*/
/*
 * 수신 데이터를 받기 위한 링버퍼 설정
 */
int SRXL2_Initialization(void){
	while(RB_init(&RC_rxRingFifo, SRXL2_RING_BUFFER_SIZE));

	return 0;
}


/*
 * 수신기와 연결하기 위한 Handshake 절차 수행
 * 텔레메트리 장치 포함 범용적 설계 필요
 *
 * + Handshake만 제대로 하면 Bind 없이 Control Packet 보내는듯.
 * 데이터시트에 의하면 Handshake 과정에 다른 패킷이 전송되면 바로 Control 패킷 보내도록 함
 */
int SRXL2_Connect(void){
	SRXL2_Handshake_Data* rx_handshake;

	while(1)
	{
		SRXL2_GetData();
		if(packet.header.pType == SRXL_HANDSHAKE_ID)
		{
			rx_handshake = &(((SRXL2_Handshake_Packet *) SRXL2_data)->data);

			// 수신기의 ID를 가져옴
			if((rx_handshake->SrcID)>>4 == 0x1)
			{
				receiver_info.SrcID = rx_handshake->SrcID;
				receiver_info.Info = rx_handshake->Info;
				receiver_info.UID = rx_handshake->UID;
				break;
			}
		}
	}

	uint8_t tx_packet_fc[14] ={
			SPEKTRUM_SRXL_ID,
			SRXL_HANDSHAKE_ID,
			0x0e,
			SRXL_FC_DEVICE_ID,				// 장치 ID. 0x30 : FC master로 설정
			receiver_info.SrcID,			// 타겟 ID. 수신기
			0x60,							// 우선 순위(높을 수록 응답 요구 빈도 증가. max=100)
			SRXL_BAUD_115200,				// Baud
			0x01,							// Info
			0x12, 0x34, 0x56, 0x78, 		// UID (32-bit).
			0x00, 0x00						// CRC. SRXL2_doHandshake에서 자동 생성함.
	};

	while(SRXL2_doHandshake((SRXL2_Handshake_Packet *)tx_packet_fc));
	// while(SRXL2_doBind());

	return 0;
}


/*
 * 수신 데이터 로딩
 */
int SRXL2_GetData(){
	SRXL2_Header *header = &packet.header;
	while(SRXL2_readByte())
	{
		if(calculate_crc(SRXL2_data, header->len) == packet.crc){
			break;
		}
	}

	switch(header->pType){
	case SRXL_HANDSHAKE_ID :
		break;
	case SRXL_CTRL_ID :
		SRXL2_SendTelemetryData();
		break;
	}
	return 0;
}



/* SRXL2.h -------------------------------------------------------------------*/
/*
 * 수신 인터럽트 IRQ 2
 * - IRQ1에서 수신 데이터 링버퍼에 저장
 * - IRQ2 해당 함수에서 링버퍼 데이터 로딩 및 SRXL2_data에 저장
 * @retval 0 : 수신 완료
 * @retval -1 : 수신 인터럽트 없음
 * @retval -2 : 링버퍼 오류
 * @retval -3 : 기타 오류
 * 			  : 링버퍼 크기 초과
 */
int SRXL2_readByte(void){
	// 단축어..
	SRXL2_Packet *rx = &packet;
	SRXL2_Header *header = &rx->header;
	enum INDEX_PACKET {
			pType = 1,
			len = 2
	};

	if(RC_rxFlag.uart == 0)
	{
		return -1;
	}
	if(RB_isempty(&RC_rxRingFifo))
	{
		return -2;
	}


	RC_rxFlag.uart = 0;

	for(uint8_t cnt = 0; cnt < SRXL2_RING_BUFFER_SIZE; cnt++){

		SRXL2_data[cnt] = RB_read(&RC_rxRingFifo);

		if(SRXL2_data[cnt] == SPEKTRUM_SRXL_ID){
			cnt = 0;
			SRXL2_data[0] = SPEKTRUM_SRXL_ID;
		}

		if(cnt>len && SRXL2_data[len] == cnt){
			header->speckrum_id = SPEKTRUM_SRXL_ID;
			header->pType = SRXL2_data[pType];
			header->len = SRXL2_data[len];

			rx->Data = SRXL2_data;
			rx->crc = ((uint16_t)SRXL2_data[header->len -2] << 8 | SRXL2_data[header->len -1]);

			// DEBUG
			// CDC_Transmit_FS(SRXL2_data, header->len);
			return 0;
		}
	}
	return -3;
}


/*
 * ControlData 정보 파싱 및 RC_Channel[]에 전달
 */
int SRXL2_parseControlData(void)
{
	return 0;
}


/*
 * 장치간 Handshake 동작 수행
 * Bus내 연결된 장치 정보 알림
 *
 * @parm SRXL2_Handshake_Packet *packet
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 * @retval -2 : 패킷 크기와 정보가 불일치
 */
int SRXL2_doHandshake(SRXL2_Handshake_Packet *tx_packet)
{
	SRXL2_Handshake_Data* rx_handshake;
	SRXL2_Handshake_Data* data = &tx_packet->data;

	uint8_t len = tx_packet->header.len;
	if(sizeof(*tx_packet) != len) return -2;

	while(1)
	{
		SRXL2_GetData();
		if(packet.header.pType == SRXL_HANDSHAKE_ID)
		{
			rx_handshake = &(((SRXL2_Handshake_Packet *) SRXL2_data)->data);

			if(rx_handshake->SrcID == data->DestID && rx_handshake->DestID == data->SrcID)
			{
				break;
			}
		}
	}

	insert_crc(tx_packet, len);
	return RC_halfDuplex_Transmit(tx_packet, len);
}


/*
 * 수신기와 Bind 동작 수행
 *
 * 기타 Bind 동작 수행 등 범용적 설계 필요
 *
 * @parm void -> SRXL2_Bind_Packet * 수정 필요
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 */
int SRXL2_doBind(void)
{
	uint8_t bind_packet[21] =
	{
		SPEKTRUM_SRXL_ID,
		SRXL_BIND_ID,
		21,
		0xEB,            // Request: Enter Bind Mode
		0x30,            // DeviceID (Receiver)
		0xA2,            // Type: DSMX 22ms
		0x01,            // Options: Telemetry 활성화
		0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,  // GUID (64-bit)
		0x12, 0x34, 0x56, 0x78,  // UID (32-bit)
		0x00, 0x00   // CRC 자리 (계산 후 입력)
	};
	insert_crc(bind_packet, sizeof(bind_packet));

	return RC_halfDuplex_Transmit(bind_packet, sizeof(bind_packet));
}


/*
 * crc 계산
 * @parm const uint8_t* data : data address
 * @parm uint8_t len : sizeof(data)
 * @retval uint16_t crc
 */
uint16_t calculate_crc(const uint8_t *data, uint8_t len)
{
	uint16_t crc = 0x0000;
	for (uint8_t i = 0; i < len-2; i++) {
		crc ^= ((uint16_t)data[i] << 8);
		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = (crc << 1);
		}
	}

	return crc;
}


/*
 * crc 계산 후 삽입
 * @parm uint8_t* data : data address
 * @parm uint8_t len : sizeof(data)
 * @retval uint16_t crc
 */
uint16_t insert_crc(uint8_t *data, uint8_t len)
{
	uint16_t crc = calculate_crc(data, len);

	data[len -2] = (uint8_t)(crc >> 8);
	data[len -1] = (uint8_t)(crc & 0xFF);

	return crc;
}
