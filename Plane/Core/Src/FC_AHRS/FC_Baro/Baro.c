/*
 * Baro.c
 *
 *  Created on: June 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_Baro/Baro_module.h>
#include <FC_Serial/MiniLink/MiniLink_module.h>


/* Functions -----------------------------------------------------------------*/
/*
 * @brief Baro 초기화
 * @detail Baro 1 - LPS22H
 * @parm none
 * @retval 0
 */
uint8_t Baro_Initialization(void)
{
	// Work in progress!
	return 0;

	LPS22HH_Initialization();

	return 0;
}


/*
 * @brief 데이터 로딩
 * @detail SCALED_PRESSURE 에 저장
 * @parm none
 * @retval none
 */
unsigned int Baro_GetData(void)
{
	// Work In Progress!
	return 0;

	// SCALED_PRESSURE
	LPS22HH_GetData();

	// SCALED_PRESSURE2

//	ComplementaryFilter();
//	KalmanFilter();

	return 0;
}


/* Functions 2 ---------------------------------------------------------------*/
/*
 * @brief 압력을 고도 정보로 변환
 * @detail SCALED_PRESSURE 에 저장
 * @parm float pressure : 현재 고도 (hPa)
 * @parm float temperature : 현재 온도 (cdegC)
 * @retval none
 */
float pascal2meter(float pressure, float temperature)
{
	return (1.0f - powf((pressure / 1013.25f), 0.1902226f)) * (temperature + 273.15f) / 0.0065f;
}
