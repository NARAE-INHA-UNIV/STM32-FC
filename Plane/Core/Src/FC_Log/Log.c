/*
 * Log.c
 *
 *  Created on: Mar 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Log/Log.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdio.h>
#include <FC_AHRS/FC_IMU/Madgwick.h> // Madgwick 함수 선언
#include <gps.h>//gps
#include <FC_AHRS/FC_IMU/IMU.h>
#include <Kalman.h>
#include "Speedmeter.h"
/* Variables -----------------------------------------------------------------*/
const uint8_t code = 0xFD;
uint16_t logType = 0;

extern float imu_roll, imu_pitch;

extern IMU_Velocity_t imu_velocity;
extern IMU_Velocity_t kalman_velocity;
/* Functions -----------------------------------------------------------------*/
//int Log_Send()
//{
//	static uint32_t previous_time = 0;
//
//	// 10Hz 단위로 전송
//	if(!(msg.system_time.time_boot_ms - previous_time > 100)) return -1;
//	previous_time = msg.system_time.time_boot_ms;
//
//	switch(logType)
//	{
//	case 26: LOG_TRANSMIT(msg.scaled_imu); break;
//	case 27: LOG_TRANSMIT(msg.raw_imu); break;
//	case 29: LOG_TRANSMIT(msg.scaled_pressure); break;
//	case 36: LOG_TRANSMIT(msg.servo_output_raw); break;
//	case 65: LOG_TRANSMIT(msg.RC_channels); break;
//	default: break;
//	}
//	return 0;
//}

int Log_Send()
{
	static uint32_t previous_time = 0;

	// 10Hz 주기
	if (!(msg.system_time.time_boot_ms - previous_time > 100)) return -1;
	previous_time = msg.system_time.time_boot_ms;

	// 원하는 데이터를 printf로 출력
	// Raw IMU 출력
//	printf("[IMU Raw] gx: %d gy: %d gz: %d | ax: %d ay: %d az: %d | temp: %d\r\n",
//	       msg.raw_imu.xgyro,
//	       msg.raw_imu.ygyro,
//	       msg.raw_imu.zgyro,
//	       msg.raw_imu.xacc,
//	       msg.raw_imu.yacc,
//	       msg.raw_imu.zacc,
//	       msg.raw_imu.temperature
//	);
	// Scaled IMU 출력
//	printf("[IMU Scaled] GX: %d GY: %d GZ: %d | AX: %d AY: %d AZ: %d\r\n",
//		msg.scaled_imu.xgyro,
//		msg.scaled_imu.ygyro,
//		msg.scaled_imu.zgyro,
//		msg.scaled_imu.xacc,
//		msg.scaled_imu.yacc,
//		msg.scaled_imu.zacc
//	);
//
    printf("\r\n===== IMU & GPS LOG =====\r\n");
    printf("[Euler]\r\n  Roll: %.2f deg\r\n  Pitch: %.2f deg\r\n", imu_roll, imu_pitch);

    printf("[IMU Velocity]\r\n  vx: %.2f m/s\r\n  vy: %.2f m/s\r\n  vz: %.2f m/s\r\n",
           imu_velocity.vx, imu_velocity.vy, imu_velocity.vz);

    printf("[Kalman Velocity]\r\n  vx: %.2f m/s\r\n  vy: %.2f m/s\r\n  vz: %.2f m/s\r\n",
           kalman_velocity.vx, kalman_velocity.vy, kalman_velocity.vz);

    printf("[GPS]\r\n  Lat: %.6f\r\n  Lon: %.6f\r\n  Alt: %.2f m\r\n  Speed: %.2f m/s\r\n  Course: %.2f deg\r\n",
           gps_latitude, gps_longitude, gps_altitude, gps_speed, gps_course);

    printf("[Airspeed]\r\n  Speed: %.2f m/s\r\n", Speedmeter_GetAirspeed());

    printf("=========================\r\n\r\n");
	return 0;
}
/*
 * cal crc는 SRXL2.c에 존재
 * 추후 RadioControl.c로 이전
 * SRXL2.c 코드 정리
 * 	- readByte 내에 cal crc 수행 후 타입에 따라 리턴
 */
//extern uint16_t calculate_crc(const uint8_t *data, uint8_t len);
//int Log_transmit(uint8_t* p, uint8_t len)
//{
//
//    uint8_t packetLen = len+sizeof(uint8_t)*3;
//    uint8_t* packet = malloc(packetLen);
//
//    memcpy(packet, &code, sizeof(uint8_t));
//    memcpy(packet + sizeof(uint8_t), p, len);
//
//    uint16_t crc = calculate_crc(packet, packetLen);
//
//    memcpy(packet + sizeof(uint8_t) + len, &crc, sizeof(uint16_t));
//
//	CDC_Transmit_FS(packet, packetLen);
//
//	for(int i=0; i<packetLen; i++)
//	{
//		while(!LL_USART_IsActiveFlag_TXE(USART2)){}
//		LL_USART_TransmitData8(USART2, packet[i]);
//		LL_USART_TransmitData8(USART3, packet[i]); //유아트 2,3으로 동시전송
//	}
//
//    free(packet);
//
//	return packetLen;
//}

//
//void USB_CDC_RxHandler(uint8_t* Buf, uint32_t Len)
//{
//	if(Len<3 || Len > 255) return;
//	if(Buf[0] != code) return;
//
//	uint16_t crc = ((uint16_t)Buf[Len -2] << 8 | Buf[Len -1]);
//	if(crc != calculate_crc(&Buf[0], (uint8_t)Len)) return;
//
//	logType = Buf[1];
//
//	return;
//}
