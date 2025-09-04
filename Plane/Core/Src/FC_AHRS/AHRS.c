/*
 * AHRS.c
 *
 *  Created on: Jul 23, 2025
 *      Author: rlawn, leecurrent04
 *      Email : (rlawn)
 *      		leecurrent04@inha.edu (leecurrent04)
 */



/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AHRS_module.h>


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

	LL_GPIO_SetOutputPin(GYRO1_NSS_GPIO_Port, GYRO1_NSS_Pin);
	LL_GPIO_SetOutputPin(GYRO2_NSS_GPIO_Port, GYRO2_NSS_Pin);
	LL_GPIO_SetOutputPin(MAG_NSS_GPIO_Port, MAG_NSS_Pin);
	LL_GPIO_SetOutputPin(BARO_NSS_GPIO_Port, BARO_NSS_Pin);

	err |= IMU_Initialization()<<0;
	err |= MAG_Initialization()<<2;
	err |= Baro_Initialization()<<4;

	LED_SetYellow(err);

	LPF_init();
//	Magwick_Initialization(&msg.attitude_quaternion);

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

	// 단위 변환 및 LPF를 거친 후의 값(LPF 보정값)
	Vector3D acc;
	Vector3D gyro; 		// 자이로 값은 칼만필터의 상태변환 행렬의 계산에 들어감
	Vector3D mag;

	// 칼만필터의 상태변수 측정값으로 들어가는 오일러각(단위 : 라디안)
	Euler angE;

	// 칼만필터의 상태변수 측정값으로 들어가는 오일러각의 쿼터니언 변환값(단위 : 정규화된 쿼터니언)
    Quaternion angQ;


	float dt = (msg.system_time.time_boot_ms - msg.attitude.time_boot_ms) / 1000.0f;
	// 단위변환
	/*
	 * 센서값이 아닌 물리량 값으로 값을 받아도 단위변환을 해 주어야 함
	 * 물리량을 꼭 (m/s^2), (rad/s), (uT)로 변환할 것
	 */
    // 가속도 센서값 → 물리량 변환(m G -> m/s^2)
	SCALED_IMU* imu = &msg.scaled_imu;
    acc.x = imu->xacc * 0.001f * 9.81;
    acc.y = imu->yacc * 0.001f * 9.81;
    acc.z = imu->zacc * 0.001f * 9.81;

    // 자이로 센서값 → 물리량 변환(m rad/s -> rad/s)
    gyro.x = imu->xgyro * 0.001f;
    gyro.y = imu->ygyro * 0.001f;
    gyro.z = imu->zgyro * 0.001f;

    // 지자계 센서값 → 물리량 변환(m gauss -> uT)
    mag.x = imu->xmag * 0.1f;
    mag.y = imu->ymag * 0.1f;
    mag.z = imu->zmag * 0.1f;


    // LPF 업데이트
    acc =  LPF_update3D(&lpf_acc, &acc);
    gyro =  LPF_update3D(&lpf_gyro, &gyro);
    mag =  LPF_update3D(&lpf_mag, &mag);


    // 가속도를 기반으로 roll, pitch 게산
    angE.roll = atan2(acc.y, acc.z);
    angE.pitch = atan2(acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2)));

    // 지자계와 roll, pitch를 기반으로 yaw 계산
    angE.yaw = AHRS_calculateYAW(mag, angE);


    angQ = AHRS_Euler2Quaternion(angE);
    angQ = AHRS_NormalizeQuaternion(angQ);

    angQ = LKF_Update(gyro, angQ, dt);
    msg.attitude_quaternion.time_boot_ms = msg.system_time.time_boot_ms;
    msg.attitude_quaternion.q1 = angQ.q0;
    msg.attitude_quaternion.q2 = angQ.q1;
    msg.attitude_quaternion.q3 = angQ.q2;
    msg.attitude_quaternion.q4 = angQ.q3;

//    angE = AHRS_Quaternion2Euler(angQ);
    msg.attitude.time_boot_ms = msg.system_time.time_boot_ms;
    msg.attitude.roll = angE.roll;
    msg.attitude.pitch = angE.pitch;
    msg.attitude.yaw = angE.yaw;


//	Madgwick_Update(&msg.attitude_quaternion, &msg.scaled_imu);
//	Madgwick_GetEuler(&msg.attitude_quaternion, &msg.attitude);

	/*
	imu_roll  -= roll_offset;
	imu_pitch -= pitch_offset;


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


int AHRS_CalibrateOffset(void)
{
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


/*
 * @brief : Hard/Soft-Iron 보정 + 기울기 보상 + yaw 계산
 * @detail : 가속도를 기반으로 roll, pitch 추정
 * @input : 지자계 LPF 보정값, 가속도 기반 roll pitch 계산값
 * @output : 가속도+지자계 기반 yaw 게산값
 */
float AHRS_calculateYAW(Vector3D mag, Euler angle)
{
	// Hard-Iron 보정 상수값
	static const float x_Hmax = 60.0f;
	static const float y_Hmax = 60.0f;
	static const float z_Hmax = 60.0f;
	static const float x_Hmin = -60.0f;
	static const float y_Hmin = -60.0f;
	static const float z_Hmin = -60.0f;

	// Hard-Iron
	static const float x_offset= (x_Hmax + x_Hmin) / (2.0);
	static const float y_offset= (y_Hmax + y_Hmin) / (2.0);
	static const float z_offset= (z_Hmax + z_Hmin) / (2.0);

	// Soft-Iron
	static const float x_scale = ((x_Hmax - x_Hmin)+(y_Hmax - y_Hmin)+(z_Hmax - z_Hmin)) / (3.0*((x_Hmax - x_Hmin)));
	static const float y_scale = ((x_Hmax - x_Hmin)+(y_Hmax - y_Hmin)+(z_Hmax - z_Hmin)) / (3.0*((y_Hmax - y_Hmin)));
	static const float z_scale = ((x_Hmax - x_Hmin)+(y_Hmax - y_Hmin)+(z_Hmax - z_Hmin)) / (3.0*((z_Hmax - z_Hmin)));

	// 보정
	mag.x = (mag.x - x_offset) * (x_scale);
	mag.y = (mag.y - y_offset) * (y_scale);
	mag.z = (mag.z - z_offset) * (z_scale);


	// 기울기 보상
    float Xh = mag.x * cos(angle.pitch) + mag.z * sin(angle.pitch);
    float Yh = mag.x * sin(angle.roll) * sin(angle.pitch) + mag.y * cos(angle.roll) - mag.z * sin(angle.roll) * cos(angle.pitch);


    // yaw값 계산
	float out = atan2(-Yh, Xh);

	// yaw값 출력
	return out;
}



/* Functions (common.h) ------------------------------------------------------*/
// 오일러 각(roll, pitch, yaw)을 쿼터니언(q0_mea, q1_mea, q2_mea, q3_mea)으로 변환
Quaternion AHRS_Euler2Quaternion(const Euler ori)
{
	Quaternion ret;

	Euler a;
	a.roll = ori.roll / 2;
	a.pitch = ori.pitch / 2;
	a.yaw = ori.yaw / 2;

    ret.q0 = cos(a.roll) * cos(a.pitch) * cos(a.yaw) + sin(a.roll) * sin(a.pitch) * sin(a.yaw);
    ret.q1 = sin(a.roll) * cos(a.pitch) * cos(a.yaw) - cos(a.roll) * sin(a.pitch) * sin(a.yaw);
    ret.q2 = cos(a.roll) * sin(a.pitch) * cos(a.yaw) + sin(a.roll) * cos(a.pitch) * sin(a.yaw);
    ret.q3 = cos(a.roll) * cos(a.pitch) * sin(a.yaw) - sin(a.roll) * sin(a.pitch) * cos(a.yaw);

    return ret;
}
// 쿼터니언을 오일러 각도로 변환해서 출력
Euler AHRS_Quaternion2Euler(const Quaternion ori)
{
	Euler ret;
#define POW(x) (x*x)
    ret.roll = RAD2DEG(atan2( 2.0f*(ori.q0 * ori.q1 + ori.q2*ori.q3), 1.0f - 2.0f*(POW(ori.q1) + POW(ori.q2)) ));
    ret.pitch = RAD2DEG(asin( 2.0f*(ori.q0 * ori.q2 - ori.q3*ori.q1) ));
    ret.yaw = RAD2DEG(atan2( 2.0f*(ori.q0*ori.q3 + ori.q1*ori.q2), 1.0f - 2.0f*(POW(ori.q2) + POW(ori.q3)) ));
#undef POW
    return ret;
}

// 쿼터니언 정규화
Quaternion AHRS_NormalizeQuaternion(const Quaternion ori)
{
	Quaternion ret;
#define POW(x) (x*x)
    float norm = sqrtf(POW(ori.q0)+POW(ori.q1)+POW(ori.q2)+POW(ori.q3));
#undef POW
    ret.q0 = ori.q0 / norm;
    ret.q1 = ori.q1 / norm;
    ret.q2 = ori.q2 / norm;
    ret.q3 = ori.q3 / norm;

    return ret;
}
