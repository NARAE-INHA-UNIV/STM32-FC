/*
 * IMU.c
 *
 *  Created on: April 30, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/IMU.h>
#include <FC_AHRS/Filter/Kalman.h>
#include <math.h>


Kalman1D_t kal_vx, kal_vy;
IMU_Velocity_t kalman_velocity = {0.0f, 0.0f, 0.0f};
IMU_Velocity_t imu_velocity = {0.0f, 0.0f, 0.0f};

extern double gps_speed;
extern double gps_course;

float imu_roll = 0.0f;
float imu_pitch = 0.0f;
float pitch_offset = 0.0f;
float roll_offset = 0.0f;
#define OFFSET_SAMPLE_COUNT 100


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
 * @detail
 * 		필터 및 오프셋 보정된 값
 * 		SCALED_IMU(_, 2,3)에 저장
 * @parm none
 * @retval none
 */
unsigned int IMU_GetData(void)
{
	IMU_getDataRaw();

//	ComplementaryFilter();
//	KalmanFilter();

    static uint32_t last_time = msg.system_time.time_boot_ms;
	float dt = (msg.system_time.time_boot_ms - last_time) / 1000.0f;
    last_time = now_time;

	//	MadgwickFilter
    float ax = (float)msg.scaled_imu.xacc / 1000.0f; // mG → G
    float ay = (float)msg.scaled_imu.yacc / 1000.0f; // mG → G
    float az = (float)msg.scaled_imu.zacc / 1000.0f; // mG → G

    float gx = (float)msg.scaled_imu.xgyro / 1000.0f; // mdeg/s → deg/s
    float gy = (float)msg.scaled_imu.ygyro / 1000.0f; // mdeg/s → deg/s
    float gz = (float)msg.scaled_imu.zgyro / 1000.0f; // mdeg/s → deg/s

    gx *= M_PI / 180.0f; // deg/s → rad/s
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);

    Madgwick_GetEuler(&imu_roll, &imu_pitch, NULL);
    imu_roll  -= roll_offset;
    imu_pitch -= pitch_offset;
    IMU_computeVelocity();

    // GPS 속도 X, Y 변환
    float course_rad = gps_course * M_PI / 180.0f;
    float gps_speed_x = gps_speed * cosf(course_rad);
    float gps_speed_y = gps_speed * sinf(course_rad);

    // Kalman 업데이트
    kalman_velocity.vx = Kalman_Update(&kal_vx, gps_speed_x, ax * 9.81f, global_dt);
    kalman_velocity.vy = Kalman_Update(&kal_vy, gps_speed_y, ay * 9.81f, global_dt);

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
        IMU_GetDataRaw();

        roll_sum  += imu_roll;
        pitch_sum += imu_pitch;

        HAL_Delay(10);      // 약 1초 동안 평균
    }

    roll_offset = roll_sum / OFFSET_SAMPLE_COUNT;
    pitch_offset = pitch_sum / OFFSET_SAMPLE_COUNT;

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
	retVal = ICM42688_GetData();

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

	return 0;
}


void IMU_computeVelocity(void)
{
    float ax = (float)msg.scaled_imu.xacc / 1000.0f * 9.81f;
    float ay = (float)msg.scaled_imu.yacc / 1000.0f * 9.81f;
    float az = (float)msg.scaled_imu.zacc / 1000.0f * 9.81f;

    float roll_rad = imu_roll * M_PI / 180.0f;
    float pitch_rad = imu_pitch * M_PI / 180.0f;

    // Body → World 회전행렬
    float R11 = cosf(pitch_rad);
    float R12 = sinf(roll_rad) * sinf(pitch_rad);
    float R13 = cosf(roll_rad) * sinf(pitch_rad);

    float R21 = 0;
    float R22 = cosf(roll_rad);
    float R23 = -sinf(roll_rad);

    float R31 = -sinf(pitch_rad);
    float R32 = sinf(roll_rad) * cosf(pitch_rad);
    float R33 = cosf(roll_rad) * cosf(pitch_rad);

    // 중력 벡터 (World frame)
    float gx_w = 0.0f;
    float gy_w = 0.0f;
    float gz_w = 9.81f;

    // 중력을 Body frame으로 변환: g_b = R^T * g_w
    float gx_b = R11 * gx_w + R21 * gy_w + R31 * gz_w;
    float gy_b = R12 * gx_w + R22 * gy_w + R32 * gz_w;
    float gz_b = R13 * gx_w + R23 * gy_w + R33 * gz_w;

    // 센서의 순수 가속도
    float ax_corrected = ax - gx_b;
    float ay_corrected = ay - gy_b;
    float az_corrected = az - gz_b;

    // 속도 적분
    imu_velocity.vx += ax_corrected * global_dt;
    imu_velocity.vy += ay_corrected * global_dt;
    imu_velocity.vz += az_corrected * global_dt;
}


void KalmanFilter(void)
{
	return;
}


void ComplementaryFilter(void)
{
	return;
}
