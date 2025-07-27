/*
 * LPF.c
 *
 *  Created on: Jul 27, 2025
 *      Author: twwawy
 *      Email : twwawy37@gmail.com
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AP_Filter/LPF.h>


/* Variables -----------------------------------------------------------------*/
LPFState lpf_acc;
LPFState lpf_gyro;
LPFState lpf_mag;


/* Functions -----------------------------------------------------------------*/
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


/*
 * @brief : 1차 저주파 통과 필터
 * @detail : x_k = a*x_(k-1) + (1-a)*x_k
 * @input : 센서값 + alpha값
 * @output : 센서값 LPF 보정값
 */
float LPF_update(float alpha, float* previous, float in)
{
    // 1차 저주파 필터 : x_k = a*x_n + (1-a)*y_(n-1)
	float out = alpha * in + (1.0f - alpha) * (*previous);

	// 이전값 갱신
	*previous = out;

	// LPF 보정값 출력
	return out;
}
