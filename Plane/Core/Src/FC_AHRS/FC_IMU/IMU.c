/*
 * IMU.c
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/IMU.h>


/* Macros --------------------------------------------------------------------*/
#define OFFSET_SAMPLE_COUNT 100


/* Functions -----------------------------------------------------------------*/
/*
 * @brief IMU 초기화
 * @detail IMU 1 - ICM42688P : GYRO, ACC, TEMP
 * @parm none
 * @retval 0 : 정상
 * 		0bNM : N - ICM42688 err
 * 		0bNM : M - BMI323 err
 */
uint8_t IMU_Initialization(void)
{
	uint8_t temp = 0;
	temp |= (ICM42688P_Initialization()<<0);
	temp |= (BMI323_Initialization()<<1);

	return 0;
}


/*
 * @brief 데이터 로딩
 * @detail
 * 		필터 및 오프셋 보정된 값
 * 		SCALED_IMU(_, 2,3)에 저장
 * @parm none
 * @retval none
 */
uint8_t IMU_GetData(void)
{
	IMU_getDataRaw();

	return 0;
}



/*
 * @brief IMU 오프셋 보정
 * @detail SCALED_IMU(2,3)에 저장
 * @parm none
 * @retval none
 */
void IMU_CalculateOffset(void)
{
	float roll_sum = 0.0f;
	float pitch_sum = 0.0f;

	for (int i = 0; i < OFFSET_SAMPLE_COUNT; i++)
	{
		IMU_getDataRaw();

//		roll_sum  += imu_roll;
//		pitch_sum += imu_pitch;

		HAL_Delay(10);      // 약 1초 동안 평균
	}

//	roll_offset = roll_sum / OFFSET_SAMPLE_COUNT;
//	pitch_offset = pitch_sum / OFFSET_SAMPLE_COUNT;

	return;
}




/* Functions -----------------------------------------------------------------*/
/*
 * @brief 데이터 RAW 로딩
 * @detail SCALED_IMU(_, 2,3)에 저장
 * @parm none
 * @retval none
 */
unsigned int IMU_getDataRaw(void)
{
	uint16_t retVal = 0;

	// SCALED_IMU
	retVal = ICM42688P_GetData();

	// SCALED_IMU2
	retVal = (BMI323_GetData() << 4);


	// SCALED_IMU3

	// Error
	if (retVal & 0x0eee)
	{
		LL_GPIO_SetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
		return 1;
	}

	LL_GPIO_ResetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);

	return 0;
}


