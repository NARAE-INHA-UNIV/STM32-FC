/*
 * Serial.c
 *
 *  Created on: Jul 13, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 *
 *  @detail :
 *  	UART, CDC를 통합 관리하는 코드
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Serial/Serial_module.h>

#include <main.h>

#include <FC_Basic/LED/LED.h>
#include <FC_AHRS/FC_IMU/IMU.h>


/* Variables -----------------------------------------------------------------*/
MiniLinkPacket serialRX;


/* Functions -----------------------------------------------------------------*/
int SERIAL_Initialization()
{
	// interrupt when finished receiving
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_RXNE(USART2);
	LL_USART_EnableIT_RXNE(USART3);
	LL_USART_EnableIT_RXNE(UART4);
	LL_USART_EnableIT_RXNE(UART5);

	jumboTx.start = (uint8_t*)malloc(sizeof(uint8_t)*LOG_BUFFER_SIZE);
	if(jumboTx.start == 0) { return 1; }
	jumboTx.offset = jumboTx.start;

	return 0;
}

int SERIAL_Handler()
{
	if(serialRX.flag.ack == 0)
	{
		MiniLink_Send();
		return 0;
	}
	if(serialRX.flag.nack == 1)
	{
		// re-ask

		// (WIP)
		serialRX.flag.ack = 0;
		serialRX.flag.nack = 0;
		return 0;
	}

	serialRX.flag.ack = 0;

	switch(serialRX.header.msgId)
	{
	case 2:
		IMU_CalibrateOffset();
		break;

	case 250:
		// 외부 제어기(각도) 제어 이득 설정
		if(serialRX.header.length-MINILINK_MIN_PACKET_SIZE != sizeof(param.pid.ANGLE)) break;
		memcpy(&param.pid.ANGLE, serialRX.payload, sizeof(param.pid.ANGLE));

		break;

	case 251:
		// 내부 제어기(각속도) 제어 이득 설정
		if(serialRX.header.length-MINILINK_MIN_PACKET_SIZE != sizeof(param.pid.RATE)) break;
		memcpy(&param.pid.RATE, serialRX.payload, sizeof(param.pid.RATE));

		break;

    case 252:
        // PID 제어 이득 설정
		if(serialRX.header.length-MINILINK_MIN_PACKET_SIZE != sizeof(param.pid)) break;
        memcpy(&param.pid, serialRX.payload, sizeof(param.pid));

        break;
	}

	return 0;
}


/*
 * @brief 수신 인터럽트 IRQ2 (UART 전용)
 * @detail
 * 		USB_CDC는 USB_CDC_RxHandler(); 참고
 * 		uint8_t *p (malloc) 메모리 할당 주의
 * @param
 * 		uint8_t serialNumber : Telem 1/2, GPS 1/2 등을 알리는 번호
 * 		uint8_t data : 1 byte rx data
 */
void SERIAL_receivedIRQ2(uint8_t serialNumber, uint8_t data)
{
	if(2 != param.serial[serialNumber].protocol)
	{
		return;
	}

	static uint8_t cnt = 0;
	static uint8_t* p;

	switch(cnt++)
	{
	case 0:
		if(MINILINK_STX_CODE != data) cnt = 0;
		break;
	case 1:
		p = (uint8_t*)malloc(sizeof(uint8_t)*data);
		p[0] = MINILINK_STX_CODE;
		p[1] = data;
		break;
	default :
		p[cnt-1] = data;

		break;
	}

	// all byte recevied
	if(cnt>=p[1])
	{
		cnt = 0;

		uint8_t len = p[MAVLINK_PACKET_INDEX_LENGTH];
		SERIAL_receviedParser(p, len);

		free(p);
	}

	return;
}


/*
 * @brief 수신 후 값 파싱
 * @detail
 * 		패킷 구조: [STX][Length][SEQ][MSG ID][Payload][CRC]
 *
 * 		@ warning
 * 		uint8_t* serial.payload (malloc)
 * @param
 * 		uint8_t serialNumber : Telem 1/2, GPS 1/2 등을 알리는 번호
 * 		uint8_t data : 1 byte rx data
 */
void SERIAL_receviedParser(uint8_t* Buf, uint32_t Len)
{
	serialRX.flag.ack = 1;
	serialRX.flag.nack = 0;

	// crc : Big endian
	uint16_t crc = ((uint16_t)Buf[Len -2] << 8 | Buf[Len -1]);

	if(crc != calculate_crc(&Buf[0], (uint8_t)Len)){
		serialRX.flag.nack = 1;
		return;
	}

	if(serialRX.payload != NULL){
		free(serialRX.payload);
		serialRX.payload = NULL;
	}

	serialRX.header.length = Buf[MAVLINK_PACKET_INDEX_LENGTH];
	serialRX.header.seq = Buf[MAVLINK_PACKET_INDEX_SEQ];
	serialRX.header.msgId = (uint16_t)Buf[MAVLINK_PACKET_INDEX_MSGID_H] << 8 | Buf[MAVLINK_PACKET_INDEX_MSGID_L];

	serialRX.payload = (uint8_t*)malloc((serialRX.header.length)*sizeof(uint8_t));
	memcpy(serialRX.payload, &Buf[MAVLINK_PACKET_INDEX_PAYLOAD], serialRX.header.length);
}


/* USB Functions -------------------------------------------------------------*/
/*
 * @brief USB CDC로 수신된 데이터를 처리하는 핸들러
 * @details
 * 		USB Full-speed (64바이트) 제약으로 인해 분할 수신된 MAVLink 패킷을 재조립합니다.
 * 		패킷 구조: [STX][Length][Data...]
* @param
* 		uint8_t* new_rx_buffer : 새로 수신된 데이터 버퍼
* 		uint32_t new_rx_len : 새로 수신된 데이터의 길이
*/
void USB_CDC_RxHandler(uint8_t* new_rx_buffer, uint32_t new_rx_len)
{
   // 수신된 데이터가 없으면 즉시 반환
   if (new_rx_buffer == NULL || new_rx_len == 0)
   {
       return;
   }

   static uint8_t* assembled_packet_buffer = NULL;
   static uint32_t total_packet_size = 0;
   static uint32_t current_received_size = 0;

   // 패킷의 시작 부분 (헤더)을 수신한 경우
   if (new_rx_buffer[MAVLINK_PACKET_INDEX_STX] == MINILINK_STX_CODE)
   {
       // 이전에 처리중이던 패킷이 있다면 메모리 해제 (오류 상황 복구)
       if (assembled_packet_buffer != NULL)
       {
           free(assembled_packet_buffer);
           assembled_packet_buffer = NULL;
       }

       total_packet_size = new_rx_buffer[MAVLINK_PACKET_INDEX_LENGTH];

       // 수신해야 할 전체 길이가 비정상적일 경우 처리 중단
       if (total_packet_size < MINILINK_MIN_PACKET_SIZE)
       {
           total_packet_size = 0;
           current_received_size = 0;
           return;
       }

       // 전체 패킷을 저장할 메모리 동적 할당
       assembled_packet_buffer = (uint8_t*)malloc(total_packet_size);
       if (assembled_packet_buffer == NULL)
       {
           // TODO: 메모리 할당 실패 로그 출력
           total_packet_size = 0;
           current_received_size = 0;
           return;
       }

       // 첫 패킷 데이터를 버퍼에 복사
       memcpy(assembled_packet_buffer, new_rx_buffer, new_rx_len);
       current_received_size = new_rx_len;
   }
   // 패킷의 중간 또는 마지막 부분을 수신한 경우
   else
   {
       // 패킷의 시작 없이 중간 데이터가 들어온 경우 (오류), 무시하고 반환
       if (assembled_packet_buffer == NULL)
       {
           return;
       }

       // 수신된 데이터를 기존 버퍼에 이어서 복사
       memcpy(assembled_packet_buffer + current_received_size, new_rx_buffer, new_rx_len);
       current_received_size += new_rx_len;
   }

   // 모든 패킷이 수신되었는지 확인
   if (current_received_size >= total_packet_size && assembled_packet_buffer != NULL)
   {
       SERIAL_receviedParser(assembled_packet_buffer, total_packet_size);

       // 사용한 메모리 해제 및 상태 변수 초기화
       free(assembled_packet_buffer);
       assembled_packet_buffer = NULL;
       total_packet_size = 0;
       current_received_size = 0;
   }
}

