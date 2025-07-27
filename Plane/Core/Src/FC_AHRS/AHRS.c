/*
 * AHRS.c
 *
 *  Created on: Jul 23, 2025
 *      Author: rlawn, leecurrent04
 *      Email : (rlawn)
 *      		leecurrent04@inha.edu (leecurrent04)
 */



/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AHRS.h>


/* Variables -----------------------------------------------------------------*/
Kalman1D_t kal_vx, kal_vy;
IMU_Velocity_t kalman_velocity = {0.0f, 0.0f, 0.0f};
IMU_Velocity_t imu_velocity = {0.0f, 0.0f, 0.0f};

extern double gps_speed;
extern double gps_course;

float imu_roll = 0.0f;
float imu_pitch = 0.0f;
float pitch_offset = 0.0f;
float roll_offset = 0.0f;


/* Functions -----------------------------------------------------------------*/
/*
 * @brief IMU, MAG, Baro Initialization
 * @detail (MAG)LIS2MDF Sensor's default mode is 3-wire SPI.
 * @param
 * 		void
 * @retval 0
 */
int AHRS_Initialization(void)
{
	uint8_t err = 0;
	err |= MAG_Initialization()<<0;		// Exec first
	err |= IMU_Initialization()<<1;
	err |= Baro_Initialization();

	Magwick_Initialization(&msg.attitude_quaternion);

	/*
	// Kalman 초기화
	Kalman_Init(&kal_vx, 0.1f, 5.0f, 1.0f);
	Kalman_Init(&kal_vy, 0.1f, 5.0f, 1.0f);
	 */

	return 0;
}


int AHRS_GetData(void)
{
	IMU_GetData();
	Baro_GetData();
	MAG_GetData();

	//	ComplementaryFilter();
	//	KalmanFilter();

	Madgwick_Update(&msg.attitude_quaternion, &msg.scaled_imu);
	Madgwick_GetEuler(&msg.attitude_quaternion, &msg.attitude);

	/*
	imu_roll  -= roll_offset;
	imu_pitch -= pitch_offset;

	float dt = (msg.system_time.time_boot_ms - msg.attitude.time_boot_ms) / 1000.0f;

	IMU_computeVelocity(dt);

	// GPS 속도 X, Y 변환
	float course_rad = gps_course * M_PI / 180.0f;
	float gps_speed_x = gps_speed * cosf(course_rad);
	float gps_speed_y = gps_speed * sinf(course_rad);

	// Kalman 업데이트
	kalman_velocity.vx = Kalman_Update(&kal_vx, gps_speed_x, msg.scaled_imu.xacc * 9.81f, dt);
	kalman_velocity.vy = Kalman_Update(&kal_vy, gps_speed_y, msg.scaled_imu.yacc * 9.81f, dt);
	 */

	return 0;
}


void AHRS_computeVelocity(float dt)
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
	imu_velocity.vx += ax_corrected * dt;
	imu_velocity.vy += ay_corrected * dt;
	imu_velocity.vz += az_corrected * dt;
}


/* Functions (common.h) ------------------------------------------------------*/
// 오일러 각(roll, pitch, yaw)을 쿼터니언(q0_mea, q1_mea, q2_mea, q3_mea)으로 변환
Quaternion AHRS_Euler2Quaternion(const Euler* angle)
{
	Quaternion ret;
	Euler a;
	a.roll = angle->roll / 2;
	a.pitch = angle->pitch / 2;
	a.yaw = angle->yaw / 2;

    ret.q0 = cos(a.roll) * cos(a.pitch) * cos(a.yaw) + sin(a.roll) * sin(a.pitch) * sin(a.yaw);
    ret.q1 = sin(a.roll) * cos(a.pitch) * cos(a.yaw) - cos(a.roll) * sin(a.pitch) * sin(a.yaw);
    ret.q2 = cos(a.roll) * sin(a.pitch) * cos(a.yaw) + sin(a.roll) * cos(a.pitch) * sin(a.yaw);
    ret.q3 = cos(a.roll) * cos(a.pitch) * sin(a.yaw) - sin(a.roll) * sin(a.pitch) * cos(a.yaw);

    return ret;
}

// 쿼터니언 정규화
Quaternion AHRS_NormalizeQuaternion(const Quaternion* ori)
{
	Quaternion ret;
#define POW(x) (x*x)
    float norm = sqrtf(POW(ori->q0)+POW(ori->q1)+POW(ori->q2)+POW(ori->q3));
#undef POW(x)
    ret.q0 = ori->q0 / norm;
    ret.q1 = ori->q1 / norm;
    ret.q2 = ori->q2 / norm;
    ret.q3 = ori->q3 / norm;

    return ret;
}
