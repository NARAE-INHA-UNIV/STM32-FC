/*
 * Servo.c
 *
 *  Created on: Mar 27, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Servo/Servo.h>


/* Variables -----------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
/*
 * @brief SERVO 초기화
 * @detail 출력 프로토콜, 주기 변경
 * @parm none
 * @retval none
 */
void SERVO_Initialization(void)
{
	SERVO* servo = &parm_servo;

	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableCounter(TIM3);
	LL_TIM_EnableCounter(TIM4);
	LL_TIM_EnableCounter(TIM5);

	SERVO_doDisarm();

	return;
}


/*
 * @brief PWM 주기 설정
 * @detail 50-490Hz까지 변경
 * @parm uint16_t hz 50-490
 * @retval 0 : 설정됨
 * @retval 1 : 주파수 범위 오설정
 */
uint8_t configurePWM(uint16_t hz)
{
	if(hz>490 || hz<50) return 1;

	TIM1->ARR = 20000;
	TIM1->PSC = 84;

	TIM3->ARR = 20000;
	TIM3->PSC = 84;

	TIM4->ARR = 20000;
	TIM4->PSC = 84;

	TIM5->ARR = 20000;
	TIM5->PSC = 84;

	return 0;
}


void SERVO_doArm(void)
{
	SERVO* servo = &parm_servo;
	configurePWM(servo->RATE);

	LL_TIM_EnableAllOutputs(TIM1);
	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_EnableAllOutputs(TIM4);
	LL_TIM_EnableAllOutputs(TIM5);

	/*
	for(uint8_t i=0; i<NUM_SERVO_CHANNELS; i++)
	{
		if(!(servo->GPIO_MASK&0x1<<i)) continue;
		// LL_TIM_CC_EnableChannel(TIMx, Channels)
	}
	*/
	return;
}


void SERVO_doDisarm(void)
{
	LL_TIM_DisableAllOutputs(TIM1);
	LL_TIM_DisableAllOutputs(TIM3);
	LL_TIM_DisableAllOutputs(TIM4);
	LL_TIM_DisableAllOutputs(TIM5);

	return;
}

/*
 * @brief SERVO 출력
 * @detail 출력 프로토콜 따라 수행
 * @parm none
 * @retval none
 */
void SERVO_control(void)
{
	calculateServoOutput();

	controlPWM();
	return;
}


/*
 * @brief 출력할 값(CCR) 계산
 * @detail
 * @parm none
 * @retval none
 */
void calculateServoOutput(void)
{
	SERVO* servo = &parm_servo;

	servo_output_raw.time_usec = system_time.time_boot_ms;

	for(uint8_t i=0; i<NUM_SERVO_CHANNELS; i++)
	{
		if(!(servo->GPIO_MASK&0x1<<i)) continue;

		servo_output_raw.servo_raw[i] = RC_channels.value[i];
	}

	return;
}


/*
 * @brief standard PWM 출력
 * @detail
 * @parm none
 * @retval none
 */
void controlPWM(void)
{
	SERVO* servo = &parm_servo;

	TIM4->CCR2 = servo_output_raw.servo_raw[0];
	TIM3->CCR1 = servo_output_raw.servo_raw[1];
	TIM3->CCR2 = servo_output_raw.servo_raw[2];
	/*
	uint32_t CCR_MAP[NUM_SERVO_CHANNELS] = {
		TIM4->CCR2, TIM3->CCR1, TIM3->CCR2, TIM4->CCR4,
		TIM5->CCR1, TIM5->CCR2, TIM5->CCR3, TIM5->CCR4,
		TIM3->CCR3, TIM3->CCR4, TIM1->CCR2, TIM1->CCR3,
	};

	for(uint8_t i=0; i<NUM_SERVO_CHANNELS; i++)
	{
		if(!(servo->GPIO_MASK&0x1<<i)) continue;
		// CCR_MAP[i] = servo_output_raw.servo_raw[i];
	}
	*/
	return;
}
