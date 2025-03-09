/*
 * FC_RC/SRXL2.c
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#include <FC_RC/SRXL2.h>

int SRXL2_Initialization(void){
	while(RB_init(&SRXL2_RingFifo, SRXL2_RING_BUFFER_SIZE));

	return 0;
}

// ReadByte and Hand Shake
int SRXL2_Connect(void){
	/*
	while(1)
	{
		SRXL2_GetData();
		if(packet.PacketType == SRXL_HANDSHAKE_ID)
		{
			if((handshakeRx.SrcID>>4) == 0x1&&(handshakeRx.DestID>>4) == 0x3)
			{
				if(0==SRXL2_doHandshake()){
					break;
				}
			}
		}
	}
	*/
	while(1)
	{
		SRXL2_GetData();
		if(packet.PacketType == SRXL_HANDSHAKE_ID)
		{
			if((handshakeRx.SrcID>>4) == 0x1&&(handshakeRx.DestID>>4) == 0x3)
			{
				if(0==SRXL2_doBind()){
					break;
				}
			}
		}
	}
	return 0;
}

int SRXL2_readByte(void){

	if((SRXL2_flag&0b01) != 1)
	{
		return -1;
	}
	if(RB_isempty(&SRXL2_RingFifo))
	{
		return -2;
	}

	SRXL2_flag &= 0b10;
	//LL_GPIO_ResetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);

	for(uint8_t cnt = 0; cnt < SRXL2_RING_BUFFER_SIZE; cnt++){

		// Ring Buffer에서 array로 가져옴
		SRXL2_data[cnt] = RB_read(&SRXL2_RingFifo);

		// PACKET이 시작함.
		if(SRXL2_data[cnt]==0xA6){
			cnt = 0;
			SRXL2_data[0]=0xA6;
		}
		if(cnt>SRXL2_INDEX_LENGTH && SRXL2_data[SRXL2_INDEX_LENGTH] == cnt){
			packet.SRXL2_ID = SPEKTRUM_SRXL_ID;
			packet.PacketType = SRXL2_data[SRXL2_INDEX_PACKET_TYPE];
			packet.Length = SRXL2_data[SRXL2_INDEX_LENGTH];
			packet.Data = SRXL2_data;
			packet.crc = ((uint16_t)SRXL2_data[packet.Length -2] << 8 | SRXL2_data[packet.Length -1]);

			// DEBUG
			// CDC_Transmit_FS(SRXL2_data, packet.Length);
			return 0;
		}
	}
	return -3;
}

int SRXL2_GetData(){
	while(SRXL2_readByte())
	{
		if(calculate_crc(SRXL2_data, packet.Length) == packet.crc){
			break;
		}
	}

	switch(packet.PacketType){
	case SRXL_HANDSHAKE_ID :
		handshakeRx.SrcID = packet.Data[3];
		handshakeRx.DestID = packet.Data[4];
		handshakeRx.Priority = packet.Data[5];
		handshakeRx.BaudRate = packet.Data[6];
		handshakeRx.Info = packet.Data[7];
		handshakeRx.UID = packet.Data[8]<<32
				| packet.Data[9]<<16
				| packet.Data[10]<<8
				| packet.Data[11];
	}
}

/*
 * 0 :
 * -1 : error
 */
int SRXL2_doHandshake(void)
{
	uint8_t handshake_packet[14] ={
			SPEKTRUM_SRXL_ID,
			SRXL_HANDSHAKE_ID,
			0x0e,
			0x30,					//src id
			handshakeRx.SrcID,		// desc id
			handshakeRx.Priority,
			SRXL_BAUD_115200,		// Baud
			0x01,					// Info
			0x12, 0x34, 0x56, 0x78, // UID (32-bit)
			0x00, 0x00
	};
	uint16_t crc = calculate_crc(handshake_packet, sizeof(handshake_packet));

	return SRXL2_Transmit(handshake_packet, sizeof(handshake_packet));
}

/*
 * 0 :
 * -1 : error
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

    // CRC 계산
    uint16_t crc = calculate_crc(bind_packet, sizeof(bind_packet));

	return SRXL2_Transmit(bind_packet, sizeof(bind_packet));
}

int SRXL2_Transmit(uint8_t *data, uint8_t len)
{
	if(SRXL2_flag!=0) return -1;

	LL_GPIO_SetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
	for(int i=0; i<len; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, data[i]);

		// while(!LL_USART_IsActiveFlag_TC(USART1));
	}
	LL_GPIO_ResetOutputPin(LED_DEBUG_GPIO_Port, LED_DEBUG_Pin);
	return 0;
}

uint16_t calculate_crc(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0x0000;
    for (uint8_t i = 0; i < length-2; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = (crc << 1);
        }
    }

    data[length -2] = (uint8_t)(crc >> 8);
    data[length -1] = (uint8_t)(crc & 0xFF);

    return crc;
}

/*
	LL_GPIO_SetOutputPin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);

	data[cnt] = RB_read(&SRXL2_RingFifo);

	// printf("%02X ", data[cnt]);

	if(cnt == data[3])
	{
		if(cnn == 0 && data[5] == 0x30)
		{
			//send_bind_request(&huart6);
			cnn = 1;
		}

		uint16_t channelMask = (data[10] << 8) | data[9];

		if(channelMask & 1)
		{
			n = 0;

			ch[1] = ((data[14] << 8) | data[13]);
			ch[4] = ((data[16] << 8) | data[15]);

			n = 1;
		}
		else if(n == 1)
		{
			ch[2] = ((data[14] << 8) | data[13]);
			ch[3] = ((data[16] << 8) | data[15]);
			ch[5] = ((data[18] << 8) | data[17]);
			ch[6] = ((data[20] << 8) | data[19]);

			n = 0;
		}

		cnt = 0;
	}
	cnt++;

	return 0;
}


void send_bind_request(UART_HandleTypeDef *huart)
{
    uint8_t bind_packet[21] =
    {
        0xA6, 0x41, 21,  // Header
        0xEB,            // Request: Enter Bind Mode
        0x30,            // DeviceID (Receiver)
        0xA2,            // Type: DSMX 22ms
        0x01,            // Options: Telemetry 활성화
        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,  // GUID (64-bit)
        0x12, 0x34, 0x56, 0x78,  // UID (32-bit)
        0x00, 0x00   // CRC 자리 (계산 후 입력)
    };

    // CRC 계산
    uint16_t crc = calculate_crc(bind_packet, 19);
    bind_packet[19] = (crc >> 8) & 0xFF;  // CRC_H
    bind_packet[20] = crc & 0xFF;         // CRC_L

    // UART로 패킷 전송
	  HAL_HalfDuplex_EnableTransmitter(&huart6);
	  HAL_UART_Transmit(&huart6, bind_packet, 21, 1);
//	  printf("BINDING");
	  HAL_HalfDuplex_EnableReceiver(&huart6);
}

*/
