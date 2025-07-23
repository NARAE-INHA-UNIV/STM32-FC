/*
 * Speedmeter.c
 *
 *  Created on: Jul 11, 2025
 *      Author: rlawn
 */
#include <math.h>
#include <stdio.h>
#include <test/Speedmeter.h>

#define MS4525_ADDR (0x28 << 1)
#define AIRSPEED_BUF_SIZE 15 // Moving Average 표본 개수

static uint16_t diff_pressure_raw = 0;
static float airspeed = 0.0f;
static uint16_t offset_raw = 8192;  // 초기 기본값

static float airspeed_buffer[AIRSPEED_BUF_SIZE] = {0};
static int airspeed_index = 0;
static float airspeed_sum = 0.0f;
static float lpf_airspeed = 0.0f;  // 최종 부드러운 값

float alpha = 0.2f;   // LPF 계수 (추천) 0.1 ~ 0.3 조절 가능

void Speedmeter_Init(void)
{
    // 필요시 추가 초기화
}

void Speedmeter_Calibrate(void)
{
	return ;
    uint32_t sum = 0;
    const int samples = 100;

    for (int i = 0; i < samples; i++)
    {
        uint8_t data[4] = {0};
//        HAL_I2C_Master_Receive(&hi2c1, MS4525_ADDR, data, 4, HAL_MAX_DELAY);
        uint16_t raw = ((data[0] & 0x3F) << 8) | data[1];
        sum += raw;
        HAL_Delay(10);
    }

    offset_raw = sum / samples;
    printf("Offset calibrated: %u\r\n", offset_raw);
}

void Speedmeter_Update(void)
{
	return ;
    uint8_t data[4] = {0};

    uint8_t retVal = 0;
//    retVal = HAL_I2C_Master_Receive(&hi2c1, MS4525_ADDR, data, 4, HAL_MAX_DELAY);
    if (retVal == HAL_OK)
    {
        uint16_t raw_pressure = ((data[0] & 0x3F) << 8) | data[1];
        diff_pressure_raw = raw_pressure;

        // Offset 적용
        int32_t corrected = raw_pressure - offset_raw + 8192;
        if (corrected < 1638) corrected = 1638;
        if (corrected > 14746) corrected = 14746;

        float dp_psi = ((float)(corrected - 1638) * 2.0f / (14746 - 1638)) - 1.0f;
        float  dp_Pa = -1.0f * dp_psi * 6894.76f;

        // ✅ Noise Floor 조건 (예: 5 Pa 이상만 유효)
        if (dp_Pa > 3.0f)
        {
            float raw_airspeed = sqrtf(2.0f * dp_Pa / 1.225f);

            // ✅ Deadzone 조건 (예: 2 m/s 이하 속도 무시)
            if (raw_airspeed < 2.0f)
                raw_airspeed = 0.0f;

            Speedmeter_Filter(raw_airspeed);
            airspeed = lpf_airspeed;
        }
        else
        {
            airspeed = 0.0f;
        }

//        printf("[DEBUG] raw: %u, corrected: %ld, dp_psi: %.3f, dp_Pa: %.1f, airspeed: %.2f m/s\r\n",
//               diff_pressure_raw, corrected, dp_psi, dp_Pa, airspeed);
    }
    else
    {
        airspeed = 0.0f;
        printf("[DEBUG] Sensor read fail → airspeed = 0\r\n");
    }
}

void Speedmeter_Filter(float raw_airspeed)
{
    // Moving Average 버퍼에서 이전 값 제거
    airspeed_sum -= airspeed_buffer[airspeed_index];

    // 새 값 추가
    airspeed_buffer[airspeed_index] = raw_airspeed;
    airspeed_sum += raw_airspeed;

    // 인덱스 순환
    airspeed_index++;
    if (airspeed_index >= AIRSPEED_BUF_SIZE)
        airspeed_index = 0;

    // Moving Average 계산
    float avg_airspeed = airspeed_sum / AIRSPEED_BUF_SIZE;

    // Low-pass filter
    lpf_airspeed = alpha * avg_airspeed + (1.0f - alpha) * lpf_airspeed;
}

float Speedmeter_GetAirspeed(void)
{
    return airspeed;
}

//오프셋 캘리브레이션 ✅
//
//Noise Floor (5 Pa 이하 무시) ✅
//
//Deadzone (2 m/s 이하 속도 무시) ✅
//
//Moving Average + LPF ✅
//
//Pixhawk 스타일 최종 처리 ✅
