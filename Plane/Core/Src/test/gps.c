


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <test/gps.h>

volatile uint8_t gps_buf[GPS_BUF_SIZE];
volatile uint16_t gps_cnt = 0;
volatile uint8_t gps_ready_flag = 0;
static uint8_t rx_data;

double gps_latitude = 0.0;
double gps_longitude = 0.0;
double gps_altitude = 0.0;
double gps_speed = 0.0;
double gps_course = 0.0;

void GPS_Init(void)
{
	return;
}

//void GPS_UART4_IRQHandler(void) //인터럽트마다 gps데이터를 받아서 버퍼에 저장 및 파싱
//{
//    if (LL_USART_IsActiveFlag_RXNE(UART4) && LL_USART_IsEnabledIT_RXNE(UART4))
//    {
//        rx_data = LL_USART_ReceiveData8(UART4);
//
//        // 버퍼에 저장
//        if (gps_cnt < GPS_BUF_SIZE - 1)
//        {
//            gps_buf[gps_cnt++] = rx_data;
//        }
//        if (rx_data == '\n')
//        {
//            gps_ready_flag = 1;
//            GPS_Parse();     // 즉시 파싱 제어용 사용가능
//            gps_cnt = 0;     // 버퍼 초기화
//        }
//    }
//}

void GPS_Parse(void)
{
    if (!gps_ready_flag) return;

    gps_ready_flag = 0; // 한 번만 파싱

    if (strstr((char *)gps_buf, "$GNGGA"))
    {
        char *token;
        char *fields[20];
        int field_count = 0;

        token = strtok((char *)gps_buf, ",");
        while (token != NULL && field_count < 20)
        {
            fields[field_count++] = token;
            token = strtok(NULL, ",");
        }

        if (field_count > 9)
        {
            // 위도
            double lat_raw = atof(fields[2]);
            int lat_deg = (int)(lat_raw / 100);
            double lat_min = lat_raw - (lat_deg * 100);
            gps_latitude = lat_deg + (lat_min / 60.0);
            if (fields[3][0] == 'S') gps_latitude = -gps_latitude;

            // 경도
            double lon_raw = atof(fields[4]);
            int lon_deg = (int)(lon_raw / 100);
            double lon_min = lon_raw - (lon_deg * 100);
            gps_longitude = lon_deg + (lon_min / 60.0);
            if (fields[5][0] == 'W') gps_longitude = -gps_longitude;

            // 고도
            gps_altitude = atof(fields[9]);
        }
    }
    else if (strstr((char *)gps_buf, "$GNRMC"))
    {
        char *token;
        char *fields[20];
        int field_count = 0;

        token = strtok((char *)gps_buf, ",");
        while (token != NULL && field_count < 20)
        {
            fields[field_count++] = token;
            token = strtok(NULL, ",");
        }

        if (field_count > 8)
        {
            // ✅ 유효성 확인
            if (fields[2][0] == 'A')
            {
                // 속도 (knots → m/s)
                gps_speed = atof(fields[7]) * 0.51444;

                // 진행방향
                gps_course = atof(fields[8]);
            }
            else
            {
                // 무효일 경우 값 초기화
                gps_speed = 0.0;
                gps_course = 0.0;
            }
        }
    }

}
