/*
 * IMU.c
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/IMU_module.h>


/* Macros --------------------------------------------------------------------*/


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
	uint8_t err = 0;
	err |= (ICM42688P_Initialization((SCALED_IMU *)&msg.scaled_imu)<<0);
	err |= (BMI323_Initialization((SCALED_IMU*)&msg.scaled_imu2)<<1);

	IMU_CalibrateOffset();
	return err;
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
void IMU_CalibrateOffset(void)
{
	ICM42688P_CalibrateOffset(10);

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
	retVal = (ICM42688P_GetData() << 0);

	// SCALED_IMU2
	retVal = (BMI323_GetData() << 4);


	// SCALED_IMU3

	return retVal;
}


