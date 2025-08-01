/*
 * LKF.c
 * FC_AHRS/AP_Filter/LKF
 *
 *  Created on: July 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AP_Filter/LKF.h>


/* Variables -----------------------------------------------------------------*/
// 수식 노이즈(Q) : 바이어스 변화량의 표준편차의 제곱
const float Q[7][7] = {
		{ 0.25, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0 },
		{ 0.0,  0.25, 0.0,  0.0,  0.0,  0.0,  0.0 },
		{ 0.0,  0.0,  0.25, 0.0,  0.0,  0.0,  0.0 },
		{ 0.0,  0.0,  0.0,  0.25, 0.0,  0.0,  0.0 },
		{ 0.0,  0.0,  0.0,  0.0,  0.25, 0.0,  0.0 },
		{ 0.0,  0.0,  0.0,  0.0,  0.0,  0.25, 0.0 },
		{ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.25 }
};

// 센서 노이즈(R) : 가속도&지자기 기반 쿼터니언 값의 표준편차의 제곱
const float R[4][4] = {
		{ 2.3, 0.0, 0.0, 0.0 },
		{ 0.0, 2.3, 0.0, 0.0 },
		{ 0.0, 0.0, 2.3, 0.0 },
		{ 0.0, 0.0, 0.0, 2.3 }
};


/* Functions -----------------------------------------------------------------*/
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
Quaternion LKF_Update(Vector3D gyro, Quaternion angQ, float dt)
{
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
    Z[0][0] = angQ.q0;
    Z[1][0] = angQ.q1;
    Z[2][0] = angQ.q2;
    Z[3][0] = angQ.q3;



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
    sv = AHRS_NormalizeQuaternion(sv);

    return sv;
}


