/*
 * config_PARM.c
 *
 *	파라미터 값을 정의하는 부분
 *	파라미터를 Flash에 저장 & PC로부터 로딩하는 코드 작성하기 전까지 사용
 *	위 기능 구현 후 삭제
 *
 * 파라미터 값들은 다음 글 참고
 * https://ardupilot.org/copter/docs/parameters.html
 *
 *  Created on: Mar 12, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#include <FC_Param/Param.h>
#include <FC_RC/RadioControl.h>


int PARM_load(void){
	PARAM_SERVO* servo = &paramServo;
	PARAM_SERVO_CH* servo_ch = paramServoCH;

	paramRc.OPTIONS = 0;
	paramRc.OVERRIDE_TIME = 0.0;
	paramRc.PROTOCOLS = (0x00);			// 수신기 없이 테스트
//	paramRc.PROTOCOLS = (0x1<<SRXL2);	// SRXL2
//	paramRc.PROTOCOLS = (0x1<<PPM);		// PPM (FS-iA6B)
	paramRc.FS_TIMEOUT = 1.0;
	paramRc.reversedMask = 0x00;

	for(int i=0; i<RC_CHANNEL_MAX; i++)
	{
		paramRcCH[i].MIN = 1000;
		paramRcCH[i].MAX = 2000;
		paramRcCH[i].TRIM = 0;
		paramRcCH[i].DZ = 0;
		paramRcCH[i].OPTION = 0;
	}

//	paramRcMap.THR = 0;		// SRXL2 값
//	paramRcMap.ROL = 1;
//	paramRcMap.PIT = 2;
//	paramRcMap.YAW = 3;
	paramRcMap.THR = 2;		// FS-iA6B용 값들
	paramRcMap.ROL = 3;
	paramRcMap.PIT = 1;
	paramRcMap.YAW = 0;

	servo->AUTO_TRIM = 0;
	servo->DSHOT_ESC = 0;
	servo->DSHOT_RATE = 0;
	servo->RATE = 50;
	servo->GPIO_MASK = 0xFF;
	servo->RC_FS_MSK = 0xFF;
	servo->_32_ENABLE = 0;

	for(int i=0; i<SERVO_CHANNEL_MAX; i++){
		servo_ch[i].FUNCTION = 0;
		servo_ch[i].MAX = 2000;
		servo_ch[i].MIN = 1000;
		servo_ch[i].TRIM = 1500;
		servo_ch[i].REVERSED = 0;
	}
	return 0;
}
