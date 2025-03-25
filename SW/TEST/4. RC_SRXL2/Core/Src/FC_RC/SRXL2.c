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
uint8_t SRXL2_data[SRXL_MAX_BUFFER_SIZE];	// packet 저장

SRXL2_Packet packet;
SRXL2_Handshake_Data receiver_info;

uint8_t SRXL_FC_DEVICE_ID = 0x30;


/* driver_SRXL2.h ------------------------------------------------------------*/
/*
 * @brief 수신 데이터를 받기 위한 링버퍼 설정
 */
int SRXL2_Initialization(void){
	while(RB_init(&RC_rxRingFifo, SRXL2_RING_BUFFER_SIZE));

	return 0;
}


/*
 * @brief 수신기와 연결
 * @detail 수신기와 연결하기 위한 Handshake 절차 수행
 */
int SRXL2_Connect(void){
	SRXL2_Header *header = &packet.header;
	SRXL2_Handshake_Data* rx;
	SRXL2_Handshake_Packet tx_packet;

	while(1)
	{
		if(SRXL2_readByte() != 0) continue;
		if(calculate_crc(SRXL2_data, header->len) != packet.crc) continue;

		switch(header->pType)
		{
		case SRXL_CTRL_ID:
			return 2;
		case SRXL_HANDSHAKE_ID:
			rx = &(((SRXL2_Handshake_Packet *) SRXL2_data)->data);

			// 수신기의 ID를 가져옴
			if((rx->SrcID)>>4 == 0x1)
			{
				receiver_info.SrcID = rx->SrcID;
				receiver_info.Info = rx->Info;
				receiver_info.UID = rx->UID;
				break;
			}
			break;
		default:
			continue;
		}

		break;
	}

	tx_packet.header.speckrum_id = SPEKTRUM_SRXL_ID;
	tx_packet.header.pType = SRXL_HANDSHAKE_ID;
	tx_packet.header.len = sizeof(SRXL2_Handshake_Packet);

	tx_packet.data.SrcID = SRXL_FC_DEVICE_ID;
	tx_packet.data.DestID = receiver_info.SrcID;
	tx_packet.data.Priority = 0x60;
	tx_packet.data.BaudRate = SRXL_BAUD_115200;
	tx_packet.data.Info = 0x01;
	tx_packet.data.UID = 0x12345678;

	tx_packet.crc = 0x0000;

	while(SRXL2_doHandshake(&tx_packet));

	return 0;
}


/*
 * 수신 데이터 로딩
 */
int SRXL2_GetData(){
	SRXL2_Header *header = &packet.header;
	do{
		if(SRXL2_readByte() !=0) continue;
	}
	while(calculate_crc(SRXL2_data, header->len) != packet.crc);

//	while(SRXL2_readByte()){
//		if(calculate_crc(SRXL2_data, header->len) == packet.crc){
//			break;
//		}
//	}

	switch(header->pType){
	case SRXL_HANDSHAKE_ID :
		break;
	case SRXL_CTRL_ID :
		SRXL2_parseControlData((SRXL2_Control_Packet*)SRXL2_data);
		// SRXL2_SendTelemetryData();
		break;
	}
	return 0;
}



/* SRXL2.h -------------------------------------------------------------------*/
/*
 * @brief 수신 인터럽트 IRQ 2
 * @detail IRQ1에서 수신 데이터 링버퍼에 저장.
 * 		   IRQ2 해당 함수에서 링버퍼 데이터 로딩 및 SRXL2_data에 저장
 * @retval 0 : 수신 완료
 * @retval -1 : 수신 인터럽트 없음
 * @retval -2 : 링버퍼 오류
 * @retval -3 : CRC 불일치
 */
int SRXL2_readByte(void){
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

	// flag clear
	RC_rxFlag.uart = 0;

	/*
	 * SRXL2_data의 인덱스를 초과하는 문제가 발생하지 않도록 유의
	 */
	for(uint8_t cnt = 0; cnt < SRXL2_RING_BUFFER_SIZE; cnt++){
		uint8_t value = RB_read(&RC_rxRingFifo);

		if(value == SPEKTRUM_SRXL_ID){
			SRXL2_data[0] = SPEKTRUM_SRXL_ID;
			break;
		}
	}

	for(uint8_t cnt = 1; cnt < SRXL_MAX_BUFFER_SIZE; cnt++){
		SRXL2_data[cnt] = RB_read(&RC_rxRingFifo);

		if(cnt>len && SRXL2_data[len] == cnt+1){
//		if(cnt>len && SRXL2_data[len] == cnt){
			break;
		}
	}

	header->speckrum_id = SPEKTRUM_SRXL_ID;
	header->pType = SRXL2_data[pType];
	header->len = SRXL2_data[len];

	rx->Data = SRXL2_data;
	rx->crc = ((uint16_t)SRXL2_data[header->len -2] << 8 | SRXL2_data[header->len -1]);

//	if(calculate_crc(SRXL2_data, header->len) != packet.crc){
//		return -3;
//	}

	return 0;
}


/*
 * @brief ControlData 파싱
 * @detail packet에서 ControlData 파싱 수행.
 *		   data 정규화 수행.
 *		   data 범위 조정(1000us~2000us), 반전, 트림, Dead-zone 적용.
 * 		   RC_Channel[]에 저장
 * @parm SRXL_Control_Pack *rx : (SRXL2_Control_Packet*)packet
 */
int SRXL2_parseControlData(SRXL2_Control_Packet *rx)
{
	PARM_RC *parm = &PARM_rc;
	RC_Channel *rc = &RC_channel;

	// if(rx->Command == SRXL_CTRL_CMD_VTX)
	// if(rx->Command == SRXL_CTRL_CMD_FWDPGM)

	uint8_t channelCnt = 0;
	for(int i=0; i<SRXL_MAX_CHANNEL; i++)
	{
		if((rx->data.mask>>i)&0x01)
		{
			uint16_t value = rx->data.values[channelCnt];
			channelCnt++;

			// RC 값 필터링 코드 작성
			value = value<SRXL_CTRL_VALUE_MIN?SRXL_CTRL_VALUE_MIN:value;
			value = value>SRXL_CTRL_VALUE_MAX?SRXL_CTRL_VALUE_MAX:value;

			// Reverse 처리
			if((parm->reversedMask>>i)&0x01)
			{
				rc->value[i] = map(value,
						SRXL_CTRL_VALUE_MIN, SRXL_CTRL_VALUE_MAX,
						parm->CHANNEL[i].MAX, parm->CHANNEL[i].MIN) + parm->CHANNEL[i].TRIM;
			}
			else{
				rc->value[i] = map(value,
						SRXL_CTRL_VALUE_MIN, SRXL_CTRL_VALUE_MAX,
						parm->CHANNEL[i].MIN, parm->CHANNEL[i].MAX) + parm->CHANNEL[i].TRIM;
			}

			// Dead-zone 처리
			if(rc->value[i]>(1500-parm->CHANNEL[i].DZ) && rc->value[i]<(1500+parm->CHANNEL[i].DZ)){
				rc->value[i] = 1500;
			}
		}
	}
	rc->mask = rx->data.mask;

	// rssi, frameLoss, Fail-safe 기능 등 구현
	switch(rx->Command){
	case SRXL_CTRL_CMD_CHANNEL:
		break;
	case SRXL_CTRL_CMD_CHANNEL_FS:
		break;
	}

	return 0;
}


/*
 * @brief 장치간 Handshake 동작 수행
 * 		  Bus내 연결된 장치 정보 알림
 *
 * @parm SRXL2_Handshake_Packet *packet
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 * @retval -2 : 패킷 크기와 정보가 불일치
 */
int SRXL2_doHandshake(SRXL2_Handshake_Packet *tx_packet)
{
	SRXL2_Handshake_Data* rx;
	SRXL2_Handshake_Data* data = &tx_packet->data;

	uint8_t len = tx_packet->header.len;
	if(sizeof(*tx_packet) != len) return -2;

	while(1)
	{
		SRXL2_GetData();
		if(packet.header.pType == SRXL_HANDSHAKE_ID)
		{
			rx = &(((SRXL2_Handshake_Packet *) SRXL2_data)->data);

			if(rx->SrcID == data->DestID && rx->DestID == data->SrcID)
			{
				break;
			}
		}
	}

	insert_crc((uint8_t*)tx_packet, len);
	return RC_halfDuplex_Transmit((uint8_t*)tx_packet, len);
}


/*
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
