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
RingFifo_t SRXL2_RingFifo;

/*
 * 0bmn
 * m - 0: 안정 상태
 *     1: 인터럽트 발생 또는 안정 상태 이후 3ms 대기
 * n - 0: 수신 인터럽트 처리
 *   - 1: 수신 인터럽트 발생
 */
uint8_t SRXL2_flag;

uint8_t SRXL2_data[SRXL_MAX_BUFFER_SIZE];
SRXL2_Packet packet;


/* driver_SRXL2.h ------------------------------------------------------------*/
/*
 * 수신 데이터를 받기 위한 링버퍼 설정
 */
int SRXL2_Initialization(void){
	while(RB_init(&SRXL2_RingFifo, SRXL2_RING_BUFFER_SIZE));

	return 0;
}

/*
 * 수신기와 연결하기 위한 Handshake 절차 수행
 * 텔레메트리 장치 포함 범용적 설계 필요
 */
int SRXL2_Connect(void){
	SRXL2_Handshake_Packet *rx_handshake;
	while(1)
	{
		SRXL2_GetData();
		if(packet.header.pType == SRXL_HANDSHAKE_ID)
		{

			rx_handshake = (SRXL2_Handshake_Packet *) SRXL2_data;
			if((rx_handshake->data.SrcID>>4) == 0x1 && (rx_handshake->data.DestID>>4) == 0x3)
			{
				break;
			}
		}
	}

	while(SRXL2_doHandshake());
	/*
	 * Handshake만 제대로 하면 Bind 없이 Control Packet 보내는듯.
	 * 데이터시트에 의하면 Handshake 과정에 다른 패킷이 전송되면 바로 Control 패킷 보내도록 함
	 */
	// while(SRXL2_doBind());

	return 0;
}


/*
 * 수신 데이터 로딩
 */
int SRXL2_GetData(){
	while(SRXL2_readByte(&packet))
	{
		if(calculate_crc(SRXL2_data, packet.header.len) == packet.crc){
			break;
		}
	}

	switch(packet.header.pType){
	case SRXL_HANDSHAKE_ID :
		break;
	}
}



/* SRXL2.h -------------------------------------------------------------------*/
/*
 * 수신 인터럽트 IRQ 2
 * - IRQ1에서 수신 데이터 링버퍼에 저장
 * - IRQ2 해당 함수에서 링버퍼 데이터 로딩 및 SRXL2_data에 저장
 * @parm SRXL2_Packet *rx header 및 crc 기록
 * @retval 0 : 수신 완료
 * @retval -1 : 수신 인터럽트 없음
 * @retval -2 : 링버퍼 오류
 * @retval -3 : 기타 오류
 * 			  : 링버퍼 크기 초과
 */
int SRXL2_readByte(SRXL2_Packet *rx){
	enum INDEX_PACKET {
			pType = 1,
			len = 2
	};

	if((SRXL2_flag&0b01) != 1)
	{
		return -1;
	}
	if(RB_isempty(&SRXL2_RingFifo))
	{
		return -2;
	}

	SRXL2_flag &= 0b10;

	for(uint8_t cnt = 0; cnt < SRXL2_RING_BUFFER_SIZE; cnt++){

		SRXL2_data[cnt] = RB_read(&SRXL2_RingFifo);

		if(SRXL2_data[cnt]==0xA6){
			cnt = 0;
			SRXL2_data[0]=0xA6;
		}

		if(cnt>len && SRXL2_data[len] == cnt){
			rx->header.speckrum_id = SPEKTRUM_SRXL_ID;
			rx->header.pType = SRXL2_data[pType];
			rx->header.len = SRXL2_data[len];

			rx->Data = SRXL2_data;
			rx->crc = ((uint16_t)SRXL2_data[rx->header.len -2] << 8 | SRXL2_data[rx->header.len -1]);

			// DEBUG
			CDC_Transmit_FS(SRXL2_data, rx->header.len);
			return 0;
		}
	}
	return -3;
}


/*
 * 장치간 Handshake 동작 수행
 * Bus내 연결된 장치 정보 알림
 *
 * 텔레메트리 장치 포함 범용적 설계 필요
 *
 * @parm void -> SRXL2_Handshake_Packet * 수정 필요
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 */
int SRXL2_doHandshake(void)
{
	SRXL2_Handshake_Packet *rx_handshake;
	rx_handshake = (SRXL2_Handshake_Packet *) SRXL2_data;

	uint8_t handshake_packet[14] ={
			SPEKTRUM_SRXL_ID,
			SRXL_HANDSHAKE_ID,
			0x0e,
			0x30,							//src id
			rx_handshake->data.SrcID,		// desc id
			rx_handshake->data.Priority,	// desc id
			SRXL_BAUD_115200,				// Baud
			0x01,							// Info
			0x12, 0x34, 0x56, 0x78, 		// UID (32-bit)
			0x00, 0x00
	};
	insert_crc(handshake_packet, sizeof(handshake_packet));

	return SRXL2_Transmit(handshake_packet, sizeof(handshake_packet));
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

	return SRXL2_Transmit(bind_packet, sizeof(bind_packet));
}

/*
 * 데이터 송신
 *
 * 디버깅을 위해 PC1 (RSSI)에 GPIO 연결함
 *
 * @parm uint8_t* data : data address
 * @parm uint8_t len : sizeof(data)
 * @retval 0 : 송신 완료
 * @retval -1 : 송신 실패
 */
int SRXL2_Transmit(uint8_t *data, uint8_t len)
{
	if(SRXL2_flag>>1!= 0) return -1;

	LL_GPIO_SetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
	for(int i=0; i<len; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, data[i]);

		// while(!LL_USART_IsActiveFlag_TC(USART1));
	}
	LL_GPIO_ResetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
	return 0;
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
 * crc 삽입
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
