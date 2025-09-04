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
		return 0;
	}

	serialRX.flag.ack = 0;

	switch(serialRX.header.msgId)
	{
	case 1:
		LED_SetRed(2);
		break;
	case 2:
		IMU_CalibrateOffset();
		break;
	case 3:
		LED_SetBlue(serialRX.payload[0]);
		break;
	case 29:
		float *tmp = (float*)serialRX.payload;
		msg.scaled_pressure.time_boot_ms = serialRX.header.length;
		msg.scaled_pressure.press_abs = tmp[0];
		msg.scaled_pressure.press_diff = tmp[1];

		break;
	case 250:
        // 메시지로 한 번에 전달할 수 있는 데이터의 개수가 14개임
        // 일단 임시로 9개로 나눠서 작성
        // 추후 원인 분석해서 개선 필요

		if(serialRX.header.length != sizeof(param.pid.ANGLE)) break;
        
        // 외부 제어기(각도) 제어 이득 설정
        memcpy(&param.pid.ANGLE, serialRX.payload, sizeof(param.pid.ANGLE));
        
        // 임시 확인용
        msg.scaled_pressure.time_boot_ms= serialRX.header.length;
        msg.scaled_pressure.press_abs = param.pid.ANGLE.pitch.kd;
        
        break;
    case 251:
        
        // 내부 제어기(각속도) 제어 이득 설정
		if(serialRX.header.length != sizeof(param.pid.RATE)) break;
        memcpy(&param.pid.RATE, serialRX.payload, sizeof(param.pid.RATE));
        
        // 임시 확인용
        msg.scaled_pressure.time_boot_ms= serialRX.header.length;
        msg.scaled_pressure.press_diff = param.pid.RATE.pitch.kd;
        
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
		if(LOG_MAVLINK_HEADER != data) cnt = 0;
		break;
	case 1:
		p = (uint8_t*)malloc(sizeof(uint8_t)*data);
		p[0] = LOG_MAVLINK_HEADER;
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

		uint8_t len = p[1];
		SERIAL_receviedParser(p, len);

		free(p);
	}

	return;
}


/*
 * @brief 수신 후 값 파싱
 * @detail
 * 		uint8_t* serial.payload (malloc) 주의
 * @param
 * 		uint8_t serialNumber : Telem 1/2, GPS 1/2 등을 알리는 번호
 * 		uint8_t data : 1 byte rx data
 */
void SERIAL_receviedParser(uint8_t* Buf, uint32_t Len)
{
	serialRX.flag.ack = 1;
	serialRX.flag.nack = 0;

	uint16_t crc = ((uint16_t)Buf[Len -2] << 8 | Buf[Len -1]);

	if(crc != calculate_crc(&Buf[0], (uint8_t)Len)){
		serialRX.flag.nack = 1;
		return;
	}

	if(serialRX.payload != NULL){
		free(serialRX.payload);
		serialRX.payload = NULL;
	}

	serialRX.header.length = Buf[1]-7;
	serialRX.header.seq = Buf[2];
	serialRX.header.msgId = (uint16_t)Buf[4] << 8 | Buf[3];

	serialRX.payload = (uint8_t*)malloc((serialRX.header.length)*sizeof(uint8_t));
	memcpy(serialRX.payload, &Buf[5], serialRX.header.length);
}


/* USB Functions -------------------------------------------------------------*/
void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
{
	if(Len<3 || Len > 255) return;
	if(Buf[0] != LOG_MAVLINK_HEADER) return;

	SERIAL_receviedParser(Buf, Len);
	return;
}

