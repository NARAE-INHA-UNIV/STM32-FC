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

#include <FC_RC/RadioControl.h>
#include <FC_RC/SRXL2.h>
#include <FC_Param/Param.h>

extern SERVO parm_servo;
extern SERVO_CH parm_servo_ch[32];

enum RC_PARM_PROTOCOL{
	All 	 = 0,
	PPM 	 = 1,
	IBUS 	 = 2,
	SBUS 	 = 3,
	SBUS_NI  = 4,
	DSM 	 = 5,
	SUMD 	 = 6,
	SRXL 	 = 7,
	SRXL2 	 = 8,
	CRSF 	 = 9 ,
	ST24 	 = 10,
	FPORT	 = 11,
	FPORT2 	 = 12,
	FastSBUS = 13,
	DroneCAN = 14,
	Ghost	 = 15,
	MAVRadio = 16,
};

int PARM_load(void){
	SERVO* servo = &parm_servo;
	SERVO_CH* servo_ch = parm_servo_ch;
	PARM_rc.PROTOCOLS = (0x1<<SRXL2);
	PARM_rc.FS_TIMEOUT = 1.0;
	PARM_rc.reversedMask = 0x00;

	for(int i=0; i<RC_CHANNEL_MAX; i++)
	{
		PARM_rc.CHANNEL[i].MIN = 1000;
		PARM_rc.CHANNEL[i].MAX = 2000;
		PARM_rc.CHANNEL[i].TRIM = 0;
		PARM_rc.CHANNEL[i].DZ = 0;
		PARM_rc.CHANNEL[i].OPTION = 0;
	}
	PARM_rc.MAP.THR = 0;
	PARM_rc.MAP.ROL = 1;
	PARM_rc.MAP.PIT = 2;
	PARM_rc.MAP.YAW = 3;

	servo->AUTO_TRIM = 0;
	servo->DSHOT_ESC = 0;
	servo->DSHOT_RATE = 0;
	servo->RATE = 50;
	servo->GPIO_MASK = 0xFF;
	servo->RC_FS_MSK = 0xFF;
	servo->_32_ENABLE = 0;

	for(int i=0; i<32; i++){
		servo_ch[i].FUNCTION = 0;
		servo_ch[i].MAX = 2000;
		servo_ch[i].MIN = 1000;
		servo_ch[i].TRIM = 1500;
		servo_ch[i].REVERSED = 0;
	}
	return 0;
}
