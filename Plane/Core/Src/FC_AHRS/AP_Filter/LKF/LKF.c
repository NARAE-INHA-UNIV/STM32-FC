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
LPFState lpf_acc;
LPFState lpf_gyro;
LPFState lpf_mag;


/* 최종 함수 -----------------------------------------------------------------*/

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
Euler LKF_Update(SCALED_IMU* imu, float dt)
{
	Euler retval;

	///// 칼만필터 외부 /////

	// 단위 변환 및 LPF를 거친 후의 값(LPF 보정값)
	Vector3D acc;
	Vector3D gyro; 		// 자이로 값은 칼만필터의 상태변환 행렬의 계산에 들어감
	Vector3D mag;

	// 칼만필터의 상태변수 측정값으로 들어가는 오일러각(단위 : 라디안)
	Euler angle;

	// 칼만필터의 상태변수 측정값으로 들어가는 오일러각의 쿼터니언 변환값(단위 : 정규화된 쿼터니언)
    Quaternion mea;


	// 단위변환
	/*
	 * 센서값이 아닌 물리량 값으로 값을 받아도 단위변환을 해 주어야 함
	 * 물리량을 꼭 (m/s^2), (rad/s), (uT)로 변환할 것
	 */
    // 가속도 센서값 → 물리량 변환(m G -> m/s^2)
    acc.x = imu->xacc/1000.0f *9.81;
    acc.y = imu->yacc/1000.0f *9.81;
    acc.z = imu->zacc/1000.0f *9.81;

    // 자이로 센서값 → 물리량 변환(m rad/s -> rad/s)
    gyro.x = imu->xgyro/1000.0f;
    gyro.y = imu->ygyro/1000.0f;
    gyro.z = imu->zgyro/1000.0f;

    // 지자계 센서값 → 물리량 변환(uT)
    mag.x = imu->xmag * 0.6f;
    mag.y = imu->ymag * 0.6f;
    mag.z = imu->zmag * 0.6f;


    // LPF 업데이트
    acc =  LPF_update3D(&lpf_acc, &acc);
    gyro =  LPF_update3D(&lpf_gyro, &gyro);
    mag =  LPF_update3D(&lpf_mag, &mag);


    // 가속도를 기반으로 roll, pitch 게산
    angle.roll = atan2(acc.y, acc.z);
    angle.pitch = atan2(acc.x, sqrt(pow(acc.y, 2) + pow(acc.z, 2)));

    // 지자계와 roll, pitch를 기반으로 yaw 계산
    angle.yaw = calculate_YAW(mag, angle);


    mea = AHRS_Euler2Quaternion(&angle);

    // 쿼터니언 정규화
    mea = AHRS_NormalizeQuaternion(&mea);


    ///// 칼만필터 내부 /////


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
	gyro.x -= X[4][0];
	gyro.y -= X[5][0];
	gyro.z -= X[6][0];


    // 상태공간 방정식 업데이트 (A[4][4])
    A[0][1] = -0.5f * gyro.x * dt;
    A[0][2] = -0.5f * gyro.y * dt;
    A[0][3] = -0.5f * gyro.z* dt;

    A[1][0] =  0.5f * gyro.x * dt;
    A[1][2] =  0.5f * gyro.z * dt;
    A[1][3] = -0.5f * gyro.y * dt;

    A[2][0] =  0.5f * gyro.y * dt;
    A[2][1] = -0.5f * gyro.z * dt;
    A[2][3] =  0.5f * gyro.x * dt;

    A[3][0] =  0.5f * gyro.z * dt;
    A[3][1] =  0.5f * gyro.y * dt;
    A[3][2] = -0.5f * gyro.x * dt;

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
    Z[0][0] = mea.q0;
    Z[1][0] = mea.q1;
    Z[2][0] = mea.q2;
    Z[3][0] = mea.q3;



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
    mat_trans(&A[0][0], &temp2[0][0], 7, 7);

    // temp3 = temp1 * temp2
    float temp3[7][7];
    mat_mul(&temp1[0][0], &temp2[0][0], &temp3[0][0], 7, 7, 7);

    // (P-_(k)) = temp3 + Q
    mat_add(&temp3[0][0], &Q[0][0], &P[0][0], 7, 7);



    // 칼만 이득 계산 : (K_(k)) = (P-_(k))*(H^(T))*( (H)*(P-_(k))*(H^(T)) + R )^(-1)
    // K[7][4] = P[7][7]*H^(T)[4][7]*( H[4][7]*P[7][7]*H^(T)[4][7] + R[4][4] )^(-1)

    // tmep11 = (H^(T))
    float temp11[7][4];
    mat_trans(&H[0][0], &temp11[0][0], 4, 7);

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
    int inv_success = mat_inverse(&temp44[0][0], &temp55[0][0], 4);
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
    Quaternion sv = {
    		.q0 = X[0][0],
    		.q1 = X[1][0],
    		.q2 = X[2][0],
    		.q3 = X[3][0]
    };
    sv = AHRS_NormalizeQuaternion(&sv);


    // 쿼터니언을 오일러 각도로 변환해서 출력
#define POW(x) (x*x)
    retval.roll = RAD2DEG(atan2( 2.0f*(sv.q0 * sv.q1 + sv.q2*sv.q3), 1.0f - 2.0f*(POW(sv.q1) + POW(sv.q2)) ));
    retval.pitch = RAD2DEG(asin( 2.0f*(sv.q0 * sv.q2 - sv.q3*sv.q1) ));
    retval.yaw = RAD2DEG(atan2( 2.0f*(sv.q0*sv.q3 + sv.q1*sv.q2), 1.0f - 2.0f*(POW(sv.q2) + POW(sv.q3)) ));
#undef POW(x)
    return retval;

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
	lpf_acc.previous.x = 0.0f;	lpf_acc.alpha.x = 0.95f;
	lpf_acc.previous.y = 0.0f;	lpf_acc.alpha.y = 0.95f;
	lpf_acc.previous.z = 0.0f;	lpf_acc.alpha.z = 0.95f;

    // 자이로 LPF 상태 초기화
	lpf_gyro.previous.x = 0.0f;	lpf_gyro.alpha.x = 0.95f;
	lpf_gyro.previous.y = 0.0f;	lpf_gyro.alpha.y = 0.95f;
	lpf_gyro.previous.z = 0.0f;	lpf_gyro.alpha.z = 0.95f;

    // 지자계 LPF 상태 초기화
	lpf_mag.previous.x = 0.0f;	lpf_mag.alpha.x = 0.95f;
	lpf_mag.previous.y = 0.0f;	lpf_mag.alpha.y = 0.95f;
	lpf_mag.previous.z = 0.0f;	lpf_mag.alpha.z = 0.95f;
}

// 1차 저주파 통과 필터
Vector3D LPF_update3D(LPFState *t0, Vector3D* in)
{
	Vector3D out;
	out.x = LPF_update(t0->alpha.x, &(t0->previous.x), in->x);
	out.y = LPF_update(t0->alpha.y, &(t0->previous.y), in->y);
	out.z = LPF_update(t0->alpha.z, &(t0->previous.z), in->z);

	return out;
}

float LPF_update(float alpha, float* previous, float in)
{
    // 1차 저주파 필터 : x_k = a*x_n + (1-a)*y_(n-1)
	float out = alpha * in + (1.0f - alpha) * (*previous);

	// 이전값 갱신
	*previous = out;

	// LPF 보정값 출력
	return out;
}


/*
 * @brief : Hard/Soft-Iron 보정 + 기울기 보상 + yaw 계산
 * @detail : 가속도를 기반으로 roll, pitch 추정
 * @input : 지자계 LPF 보정값, 가속도 기반 roll pitch 계산값
 * @output : 가속도+지자계 기반 yaw 게산값
 */
float calculate_YAW(Vector3D mag, Euler angle)
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



