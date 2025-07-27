/*
 * LKF.c
 * FC_AHRS/AP_Filter/LKF
 *
 *  Created on: July 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */


#include <FC_AHRS/AP_Filter/LKF/LKF.h>


// 전역 구조체 인스턴스 선언
LPFState acc_phy_x,  acc_phy_y,  acc_phy_z;
LPFState gyro_phy_x, gyro_phy_y, gyro_phy_z;
LPFState mag_phy_x,  mag_phy_y,  mag_phy_z;


/* 최종 함수 -----------------------------------------------------------------*/

// 전역 출력값
float roll_kf, pitch_kf, yaw_kf;


/*
 * @brief : IMU 필터
 * @detail : 9축 IMU 센서 값을 roll, pitch, yaw 값으로 변환&보정함
 * 			 +참고로 필터의 샘플링 타임이 2ms이하일 때 필터 성능이 최고이다.
 * @input : 가속도, 자이로, 지자계 센서값, dt(반복문의 sampling time)
 * @output : roll, pitch, yaw 보정값
 *
 * 아래 계수값 3개는 튜닝 해야 함
 * Hard-Iron 보정 상수값 : 실측해야 함
 * 수식 노이즈(Q) : 바이어스 변화량의 표준편차의 제곱
 * 센서 노이즈(R) : 가속도&지자기 기반 쿼터니언 값의 표준편차의 제곱
 */
void LKF_Update(SCALED_IMU* imu, float dt)
{
	///// 칼만필터 외부 /////

	// 단위변환을 거친 후의 값(물리량 변환값)
	float x_acc_phy,  y_acc_phy,  z_acc_phy;
	float x_gyro_phy, y_gyro_phy, z_gyro_phy;
	float x_mag_phy,  y_mag_phy,  z_mag_phy;

	// LPF를 거친 후의 값(LPF 보정값)
	float x_acc, y_acc, z_acc;
	float x_gyro_mea, y_gyro_mea, z_gyro_mea;  // 자이로 값은 칼만필터의 상태변환 행렬의 계산에 들어감
	float x_mag, y_mag, z_mag;

	// 칼만필터의 상태변수 측정값으로 들어가는 오일러각(단위 : 라디안)
	float roll, pitch, yaw;

	// 칼만필터의 상태변수 측정값으로 들어가는 오일러각의 쿼터니언 변환값(단위 : 정규화된 쿼터니언)
	float q0_mea, q1_mea, q2_mea, q3_mea;


	// 단위변환
	/*
	 * 센서값이 아닌 물리량 값으로 값을 받아도 단위변환을 해 주어야 함
	 * 물리량을 꼭 (m/s^2), (rad/s), (uT)로 변환할 것
	 */
    // 가속도 센서값 → 물리량 변환(m G -> m/s^2)
    x_acc_phy = imu->xacc/1000.0f *9.81;
    y_acc_phy = imu->yacc/1000.0f *9.81;
    z_acc_phy = imu->zacc/1000.0f *9.81;

    // 자이로 센서값 → 물리량 변환(m rad/s -> rad/s)
    x_gyro_phy = imu->xgyro/1000.0f;
    y_gyro_phy = imu->ygyro/1000.0f;
    z_gyro_phy = imu->zgyro/1000.0f;

    // 지자계 센서값 → 물리량 변환(uT)
    x_mag_phy = imu->xmag * 0.6f;
    y_mag_phy = imu->ymag * 0.6f;
    z_mag_phy = imu->zmag * 0.6f;


    // LPF 업데이트
    x_acc = LPF_update(&acc_phy_x, x_acc_phy);
    y_acc = LPF_update(&acc_phy_y, y_acc_phy);
    z_acc = LPF_update(&acc_phy_z, z_acc_phy);

    x_gyro_mea = LPF_update(&gyro_phy_x, x_gyro_phy);
    y_gyro_mea = LPF_update(&gyro_phy_y, y_gyro_phy);
    z_gyro_mea = LPF_update(&gyro_phy_z, z_gyro_phy);

    x_mag = LPF_update(&mag_phy_x, x_mag_phy);
    y_mag = LPF_update(&mag_phy_y, y_mag_phy);
    z_mag = LPF_update(&mag_phy_z, z_mag_phy);


    // 가속도를 기반으로 roll, pitch 게산
    roll = atan2(y_acc, z_acc);
    pitch = atan2(x_acc, sqrt(pow(y_acc, 2) + pow(z_acc, 2)));

    // 지자계와 roll, pitch를 기반으로 yaw 계산
    yaw = YAW(x_mag, y_mag, z_mag, roll, pitch);


    // 오일러 각(roll, pitch, yaw)을 쿼터니언(q0_mea, q1_mea, q2_mea, q3_mea)으로 변환
    q0_mea = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q1_mea = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    q2_mea = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    q3_mea = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

    // 쿼터니언 정규화
    float norm1 = sqrtf( q0_mea*q0_mea + q1_mea*q1_mea + q2_mea*q2_mea + q3_mea*q3_mea );
    q0_mea /= norm1;  q1_mea /= norm1;  q2_mea /= norm1;  q3_mea /= norm1;


    ///// 칼만필터 내부 /////

    // 자이로 bias 보정값
    float x_gyro, y_gyro, z_gyro;


	// 상태변수(x)
    static float X[7][1] = {
        { 1.0 },
        { 0.0 },
        { 0.0 },
        { 0.0 },
        { 0.0 },
        { 0.0 },
        { 0.0 }
    };

	// 오차공분산 행렬(P)
	static float P[7][7] = {
	    { 0.001, 0.0,   0.0,   0.0,   0.0,   0.0,   0.0   },
	    { 0.0,   0.001, 0.0,   0.0,   0.0,   0.0,   0.0   },
	    { 0.0,   0.0,   0.001, 0.0,   0.0,   0.0,   0.0   },
	    { 0.0,   0.0,   0.0,   0.001, 0.0,   0.0,   0.0   },
	    { 0.0,   0.0,   0.0,   0.0,   0.001, 0.0,   0.0   },
	    { 0.0,   0.0,   0.0,   0.0,   0.0,   0.001, 0.0   },
	    { 0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.001 }
	};

	// 자이로센서 측정값(z_(k))
	float Z[4][1] = {
	    { 0.0 },
	    { 0.0 },
	    { 0.0 },
	    { 0.0 }
	};

	// 상태변환 행렬(A)
	float A[7][7] = {
	    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 }
	};

	// 관측 행렬(H)
	static const float H[4][7] = {
	    { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 }
	};

	// 수식 노이즈(Q) : 바이어스 변화량의 표준편차의 제곱
	static const float Q[7][7] = {
	    { 0.25, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0 },
	    { 0.0,  0.25, 0.0,  0.0,  0.0,  0.0,  0.0 },
	    { 0.0,  0.0,  0.25, 0.0,  0.0,  0.0,  0.0 },
	    { 0.0,  0.0,  0.0,  0.25, 0.0,  0.0,  0.0 },
	    { 0.0,  0.0,  0.0,  0.0,  0.25, 0.0,  0.0 },
	    { 0.0,  0.0,  0.0,  0.0,  0.0,  0.25, 0.0 },
	    { 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.25 }
	};

	// 센서 노이즈(R) : 가속도&지자기 기반 쿼터니언 값의 표준편차의 제곱
	static const float R[4][4] = {
	    { 2.3, 0.0, 0.0, 0.0 },
	    { 0.0, 2.3, 0.0, 0.0 },
	    { 0.0, 0.0, 2.3, 0.0 },
	    { 0.0, 0.0, 0.0, 2.3 }
	};

	// 칼만이득(K)
	static float K[7][4] = {
	    { 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0 },
	    { 0.0, 0.0, 0.0, 0.0 }
	};


	// 자이로 bias 보정 : gyro = gyro_mea - bias
	x_gyro = x_gyro_mea - X[4][0];
	y_gyro = y_gyro_mea - X[5][0];
	z_gyro = z_gyro_mea - X[6][0];


    // 상태공간 방정식 업데이트 (A[4][4])
    A[0][1] = -0.5f * x_gyro * dt;
    A[0][2] = -0.5f * y_gyro * dt;
    A[0][3] = -0.5f * z_gyro * dt;

    A[1][0] =  0.5f * x_gyro * dt;
    A[1][2] =  0.5f * z_gyro * dt;
    A[1][3] = -0.5f * y_gyro * dt;

    A[2][0] =  0.5f * y_gyro * dt;
    A[2][1] = -0.5f * z_gyro * dt;
    A[2][3] =  0.5f * x_gyro * dt;

    A[3][0] =  0.5f * z_gyro * dt;
    A[3][1] =  0.5f * y_gyro * dt;
    A[3][2] = -0.5f * x_gyro * dt;

    // 상태공간 방정식 업데이트 (B[4][3])
    A[0][4] =  0.5f * X[1][0] * dt;
    A[0][5] =  0.5f * X[2][0] * dt;
    A[0][6] =  0.5f * X[3][0] * dt;

    A[1][4] = -0.5f * X[0][0] * dt;
    A[1][5] =  0.5f * X[3][0] * dt;
    A[1][6] = -0.5f * X[2][0] * dt;

    A[2][4] = -0.5f * X[3][0] * dt;
    A[2][5] = -0.5f * X[0][0] * dt;
    A[2][6] =  0.5f * X[1][0] * dt;

    A[3][4] =  0.5f * X[2][0] * dt;
    A[3][5] = -0.5f * X[1][0] * dt;
    A[3][6] = -0.5f * X[0][0] * dt;


    // 측정값 업데이트
    Z[0][0] = q0_mea;
    Z[1][0] = q1_mea;
    Z[2][0] = q2_mea;
    Z[3][0] = q3_mea;



    // 상태변수 추정 : (x-_(k)) = (A)*(x_(k-1))
    // X[7][1] = A[7][7]*X[7][1]

    // temp = (A) * (x_(k-1))
    float temp[7][1];
    mat_mul(&A[0][0], &X[0][0], &temp[0][0], 7, 7, 1);

    // x-_(k)) = temp
    mat_copy(&temp[0][0], &X[0][0], 7, 1);



    // 오차공분산 추정 : (P-_(k)) = (A)*(P_(k-1))*(A^(T)) + Q
    // P[7][7] = A[7][7]*P[7][7]*A^(T)[7][7] + Q[7][7]

    // temp1 = (A) * (P_(k-1))
    float temp1[7][7];
    mat_mul(&A[0][0], &P[0][0], &temp1[0][0], 7, 7, 7);

    // temp2 = (A^(T))
    float temp2[7][7];
    mat_trp(&A[0][0], &temp2[0][0], 7, 7);

    // temp3 = temp1 * temp2
    float temp3[7][7];
    mat_mul(&temp1[0][0], &temp2[0][0], &temp3[0][0], 7, 7, 7);

    // (P-_(k)) = temp3 + Q
    mat_add(&temp3[0][0], &Q[0][0], &P[0][0], 7, 7);



    // 칼만 이득 계산 : (K_(k)) = (P-_(k))*(H^(T))*( (H)*(P-_(k))*(H^(T)) + R )^(-1)
    // K[7][4] = P[7][7]*H^(T)[4][7]*( H[4][7]*P[7][7]*H^(T)[4][7] + R[4][4] )^(-1)

    // tmep11 = (H^(T))
    float temp11[7][4];
    mat_trp(&H[0][0], &temp11[0][0], 4, 7);

    // temp22 = (P-_(k)) * temp11
    float temp22[7][4];
    mat_mul(&P[0][0], &temp11[0][0], &temp22[0][0], 7, 7, 4);

    // temp33 = (H) * temp22
    float temp33[4][4];
    mat_mul(&H[0][0], &temp22[0][0], &temp33[0][0], 4, 7, 4);

    // temp44 = temp33 + (R)
    float temp44[4][4];
    mat_add(&temp33[0][0], &R[0][0], &temp44[0][0], 4, 4);

    // temp55 = (temp44^(-1))
    float temp55[4][4];
    int inv_success = mat_inv(&temp44[0][0], &temp55[0][0], 4);
    if (inv_success == 1)
        // (K_(k)) = temp22 * temp55
        mat_mul(&temp22[0][0], &temp55[0][0], &K[0][0], 7, 4, 4);
//  else
//		printf("Warning: skipping K update\n");



    // 상태변수 계산 : (x_(k)) = (x-_(k)) + (K_(k))*( (z_(k)) - (H)*(x-_(k)) )
    // X[7][1] = X[7][1] + K[7][7]*(z[7][1] - H[7][7]*X[7][1])

    // temp111 = (H) * (x-_(k))
    float temp111[4][1];
    mat_mul(&H[0][0], &X[0][0], &temp111[0][0], 4, 7, 1);

    // temp222 = (z_(k)) - temp111
    float temp222[4][1];
    mat_sub(&Z[0][0], &temp111[0][0], &temp222[0][0], 4, 1);

    // temp333 = (K_(k)) * temp222
    float temp333[7][1];
    mat_mul(&K[0][0], &temp222[0][0], &temp333[0][0], 7, 4, 1);

    // temp444 = (x-_(k)) + temp333
    float temp444[7][1];
    mat_add(&X[0][0], &temp333[0][0], &temp444[0][0], 7, 1);

    // (x_(k)) = temp444
    mat_copy(&temp444[0][0], &X[0][0], 7, 1);



    // 오차공분산 갱신 : (P_(k)) = (P-_(k)) - (K_(k))*(H)*(P-_(k))
    // P[7][7] = P[7][7] - K[7][7]*H[7][7]*P[7][7]

    // temp1111 = (K_(k)) * (H)
    float temp1111[7][7];
    mat_mul(&K[0][0], &H[0][0], &temp1111[0][0], 7, 4, 7);

    // temp2222 = temp1111 * (P-_(k))
    float temp2222[7][7];
    mat_mul(&temp1111[0][0], &P[0][0], &temp2222[0][0], 7, 7, 7);

    // temp3333 = (P-_(k)) - temp2222
    float temp3333[7][7];
    mat_sub(&P[0][0], &temp2222[0][0], &temp3333[0][0], 7, 7);

    // (P_(k)) = temp3333
    mat_copy(&temp3333[0][0], &P[0][0], 7, 7);



    // 쿼터니언 정규화
    float norm2 = sqrtf( X[0][0]*X[0][0] + X[1][0]*X[1][0] + X[2][0]*X[2][0] + X[3][0]*X[3][0] );
    X[0][0] /= norm2;  X[1][0] /= norm2;  X[2][0] /= norm2;  X[3][0] /= norm2;


    // 쿼터니언을 오일러 각도로 변환해서 출력
    roll_kf = atan2( 2.0f*(X[0][0]*X[1][0] + X[2][0]*X[3][0]), 1.0f - 2.0f*(X[1][0]*X[1][0] + X[2][0]*X[2][0]) ) * (180.0 / 3.141592);
    pitch_kf = asin( 2.0f*(X[0][0]*X[2][0] - X[3][0]*X[1][0]) ) * (180.0 / 3.141592);
    yaw_kf = atan2( 2.0f*(X[0][0]*X[3][0] + X[1][0] * X[2][0]), 1.0f - 2.0f*(X[2][0]*X[2][0] + X[3][0]*X[3][0]) ) * (180.0 / 3.141592);
}



/* 내부 함수 ----------------------------------------------------------------------------------*/

/*
 * @brief : 1차 저주파 통과 필터
 * @detail : x_k = a*x_(k-1) + (1-a)*x_k
 * @input : 센서값 + alpha값
 * @output : 센서값 LPF 보정값
 */

// LPF 상태 초기화
void LPF_init(void)
{
    // 가속도 LPF 상태 초기화
    acc_phy_x.phy_pre  = 0.0f;  acc_phy_x.alpha  = 0.95f;
    acc_phy_y.phy_pre  = 0.0f;  acc_phy_y.alpha  = 0.95f;
    acc_phy_z.phy_pre  = 0.0f;  acc_phy_z.alpha  = 0.95f;

    // 자이로 LPF 상태 초기화
    gyro_phy_x.phy_pre = 0.0f;  gyro_phy_x.alpha = 0.95f;
    gyro_phy_y.phy_pre = 0.0f;  gyro_phy_y.alpha = 0.95f;
    gyro_phy_z.phy_pre = 0.0f;  gyro_phy_z.alpha = 0.95f;

    // 지자계 LPF 상태 초기화
    mag_phy_x.phy_pre  = 0.0f;  mag_phy_x.alpha  = 0.95f;
    mag_phy_y.phy_pre  = 0.0f;  mag_phy_y.alpha  = 0.95f;
    mag_phy_z.phy_pre  = 0.0f;  mag_phy_z.alpha  = 0.95f;
}

// 1차 저주파 통과 필터
float LPF_update(LPFState *str, float in)
{
    // 1차 저주파 필터 : x_k = a*x_(k-1) + (1-a)*x_k
	float out = str->alpha * in + (1.0f - str->alpha) * str->phy_pre;

	// 이전값 갱신
	str->phy_pre = out;

	// LPF 보정값 출력
	return out;
}


/*
 * @brief : Hard/Soft-Iron 보정 + 기울기 보상 + yaw 계산
 * @detail : 가속도를 기반으로 roll, pitch 추정
 * @input : 지자계 LPF 보정값, 가속도 기반 roll pitch 계산값
 * @output : 가속도+지자계 기반 yaw 게산값
 */
float YAW(float x_mag, float y_mag, float z_mag, float roll, float pitch)
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
	x_mag = (x_mag - x_offset) * (x_scale);
	y_mag = (y_mag - y_offset) * (y_scale);
	z_mag = (z_mag - z_offset) * (z_scale);


	// 기울기 보상
    float Xh = x_mag*cos(pitch) + z_mag*sin(pitch);
    float Yh = x_mag*sin(roll)*sin(pitch) + y_mag*cos(roll) - z_mag*sin(roll)*cos(pitch);


    // yaw값 계산
	float out = atan2(-Yh, Xh);

	// yaw값 출력
	return out;
}



