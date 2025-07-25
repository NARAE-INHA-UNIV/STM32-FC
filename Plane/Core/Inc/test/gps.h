#ifndef INC_TEST_GPS_H_
#define INC_TEST_GPS_H_

#include "main.h"

// 버퍼 크기
#define GPS_BUF_SIZE 256

extern volatile uint8_t gps_buf[GPS_BUF_SIZE];
extern volatile uint16_t gps_cnt;
extern volatile uint8_t gps_ready_flag;

extern double gps_latitude;
extern double gps_longitude;
extern double gps_altitude;
extern double gps_speed;
extern double gps_course;

void GPS_Init(void);
//void GPS_UART4_IRQHandler(void);
void GPS_Parse(void);

#endif /* INC_TEST_GPS_H_ */
