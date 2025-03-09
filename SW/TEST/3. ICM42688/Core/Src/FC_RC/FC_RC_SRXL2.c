/*
 * RC_SRXL2.c
 *
 *  Created on: Mar 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

/*
uint16_t calculate_crc(uint8_t *data, uint8_t length)
{
    uint16_t crc = 0x0000;
    for (uint8_t i = 0; i < length; i++) {
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
