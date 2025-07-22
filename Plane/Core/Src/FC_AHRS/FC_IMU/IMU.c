/*
 * IMU.c
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/IMU.h>
#include <GCS_MiniLink/GCS_MiniLink.h>


/* Functions -----------------------------------------------------------------*/
/*
 * @brief IMU 초기화
 * @detail IMU 1 - ICM42688P : GYRO, ACC, TEMP
 * @parm none
 * @retval 0
 *         -1 : err
 */
int IMU_Initialization(void)
{
	uint8_t temp = 0;
	LL_GPIO_SetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);

	temp |= (ICM42688_Initialization()<<0);
//	temp |= (BMI323_Initialization()<<4);

	if(temp!=0)
	{
		// send error code
		return -1;
	}

	LL_GPIO_ResetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	return 0;
}


/*
 * @brief 데이터 로딩
 * @detail SCALED_IMU(2,3)에 저장
 * @parm none
 * @retval none
 */
unsigned int IMU_GetData(void)
{
	uint16_t retVal = 0;

	// SCALED_IMU
	ICM42688_GetData();

	// SCALED_IMU2
//	retVal = (BMI323_GetData() << 4);


	// SCALED_IMU3

	// Error
	if (retVal & 0x0eee)
	{
		LL_GPIO_SetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
		return 1;
	}

	LL_GPIO_ResetOutputPin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);

//	ComplementaryFilter();
//	KalmanFilter();

	return 0;
}


/* Functions -----------------------------------------------------------------*/
void KalmanFilter(void)
{
	return;
}


void ComplementaryFilter(void)
{
	return;
}
