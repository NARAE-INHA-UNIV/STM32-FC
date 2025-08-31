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


#include <FC_Serial/MiniLink/MiniLink.h>



void setRC_None(void);
void setRC_SRXL2(void);
void setRC_PPM(void);

int PARM_load(void){
	setRC_None();
//	setRC_SRXL2();
//	setRC_PPM();

	param.servo.AUTO_TRIM = 0;
	param.servo.DSHOT_ESC = 0;
	param.servo.DSHOT_RATE = 0;
	param.servo.RATE = 50;
	param.servo.GPIO_MASK = 0x0FFF;
	param.servo.RC_FS_MSK = 0xFF;
	param.servo._32_ENABLE = 0;

	for(int i=0; i<4; i++){
		param.serial[i].baud = 57;
		param.serial[i].protocol = 2;
		param.serial[i].options = 0;
	}
	for(int i=0; i<sizeof(param.servo.channel)/sizeof(param.servo.channel[0]); i++){
		param.servo.channel[i].FUNCTION = 0;
		param.servo.channel[i].MAX = 2000;
		param.servo.channel[i].MIN = 1000;
		param.servo.channel[i].TRIM = 1500;
		param.servo.channel[i].REVERSED = 0;
	}
	return 0;
}

void setRC_None(void)
{
	param.rc.OPTIONS = 0;
	param.rc.OVERRIDE_TIME = 0.0;
	param.rc.PROTOCOLS = (0x00);			// 수신기 없이 테스트
	param.rc.FS_TIMEOUT = 1.0;
	param.rc.reversedMask = 0x00;

	for(int i=0; i<sizeof(param.rc.channel)/sizeof(param.rc.channel[0]); i++)
	{
		param.rc.channel[i].MIN = 1000;
		param.rc.channel[i].MAX = 2000;
		param.rc.channel[i].TRIM = 0;
		param.rc.channel[i].DZ = 0;
		param.rc.channel[i].OPTION = 0;
	}
	param.rc.map.THR = 0;		// Default
	param.rc.map.ROL = 1;
	param.rc.map.PIT = 2;
	param.rc.map.YAW = 3;

	return;
}

void setRC_SRXL2(void)
{
	param.rc.PROTOCOLS = (0x1<<RC_PROTOCOL_SRXL2);	// SRXL2
	return;
}

void setRC_PPM(void)
{
	param.rc.PROTOCOLS = (0x1<<RC_PROTOCOL_PPM);		// PPM (FS-iA6B)
	param.rc.map.THR = 2;		// FS-iA6B용 값들
	param.rc.map.ROL = 0;
	param.rc.map.PIT = 1;
	param.rc.map.YAW = 3;
}
