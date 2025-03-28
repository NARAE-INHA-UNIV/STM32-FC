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
/*
 * 0x[타이머][채널]
 * 타미머 : 1,3-5
 * 채널 : 1-4
 */
const uint8_t SERVO_TIMER_MAP[SERVO_CHANNEL_MAX] = {
		0x42, 0x31, 0x32, 0x44,
		0x51, 0x52, 0x53, 0x54,
		0x33, 0x34, 0x12, 0x14
};


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

	TIM1->ARR = 1000000/hz-1;
	TIM1->PSC = 84-1;

	TIM3->ARR = 20000;
	TIM3->PSC = 84-1;

	TIM4->ARR = 20000;
	TIM4->PSC = 84-1;

	TIM5->ARR = 20000;
	TIM5->PSC = 84-1;

	return 0;
}


void SERVO_doArm(void)
{
	SERVO* servo = &parm_servo;
	configurePWM(servo->RATE);

	for(uint8_t i=0; i<NUM_SERVO_CHANNELS; i++)
	{
		if(!(servo->GPIO_MASK&0x1<<i)) continue;

		TIM_TypeDef* timer;
		uint32_t ch;
		switch(SERVO_TIMER_MAP[i]>>4){
		case 1:
			timer = TIM1;
			break;
		case 3:
			timer = TIM3;
			break;
		case 4:
			timer = TIM4;
			break;
		case 5:
			timer = TIM5;
			break;
		}
		switch(SERVO_TIMER_MAP[i]&0x0F){
		case 1:
			ch = LL_TIM_CHANNEL_CH1;
			break;
		case 2:
			ch = LL_TIM_CHANNEL_CH2;
			break;
		case 3:
			ch = LL_TIM_CHANNEL_CH3;
			break;
		case 4:
			ch = LL_TIM_CHANNEL_CH4;
			break;
		}

		LL_TIM_CC_EnableChannel(timer, ch);
	}

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

	for(uint8_t i=0; i<NUM_SERVO_CHANNELS; i++)
	{
		if(!(servo->GPIO_MASK&0x1<<i)) continue;

		TIM_TypeDef* timer;
		switch(SERVO_TIMER_MAP[i]>>4){
		case 1:
			timer = TIM1;
			break;
		case 3:
			timer = TIM3;
			break;
		case 4:
			timer = TIM4;
			break;
		case 5:
			timer = TIM5;
			break;
		}
		switch(SERVO_TIMER_MAP[i]&0x0F){
		case 1:
			timer->CCR1 = servo_output_raw.servo_raw[i];
			break;
		case 2:
			timer->CCR2 = servo_output_raw.servo_raw[i];
			break;
		case 3:
			timer->CCR3 = servo_output_raw.servo_raw[i];
			break;
		case 4:
			timer->CCR4 = servo_output_raw.servo_raw[i];
			break;
		}
	}

	return;
}
