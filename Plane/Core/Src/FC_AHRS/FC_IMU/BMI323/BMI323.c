/*
 ******************************************************************************
 *
 * BMI323.c (Work In Progress!)
 * FC_AHRS/FC_IMU/BMI323/BMI323.c
 *
 ******************************************************************************
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 *
 ******************************************************************************
 *
 *  SPI Configuration
 *  CPOL = 1 (High) and CPHA = 1 (2nd)
 *
 ******************************************************************************
 */

#include <FC_AHRS/FC_IMU/BMI323/BMI323.h>


/* Functions -----------------------------------------------------------------*/
/*
 * @brief 초기 설정
 * @detail SPI 연결 수행, 감도 설정, offset 제거
 * @retval 0 : 완료
 *         -1 : 센서 에러
 */
uint8_t BMI323_Initialization(void)
{

	/*
	 * datasheet p.15
	 * who am i
	 * power check
	 */

	uint16_t temp = 0;

	if(!LL_SPI_IsEnabled(SPI3)){
		LL_SPI_Enable(SPI3);
	}
	CHIP_DESELECT();

	// Testing communication and initializing the device
	temp |= (BMI323_initTry(CHIP_ID, 0x43)<<0);

	// Checking the correct initialization status
	temp |= (BMI323_initTry(ERR_REG, 0x00)<<1);

	temp |= (BMI323_initTry(STATUS, 0x01)<<2);


//	 Configure the device for normal power mode
	// Configure the device for high performance power mode
//	temp |= (BMI323_Writebyte_S(ACC_CONF, 0x4027)<<3);
	temp |= (BMI323_Writebyte_S(ACC_CONF, 0x70A9)<<3);
	delay_us(20);

//	temp |= (BMI323_Writebyte_S(GYR_CONF, 0x404B)<<4);
	temp |= (BMI323_Writebyte_S(GYR_CONF, 0x70C9)<<4);
	delay_us(20);

	if(temp) return temp;
	// Remove Gyro X offset

	return 0;
}


/*
 * @brief 데이터 로드
 * @detail 자이로, 가속도 및 온도 데이터 로딩, 물리량 변환
 * @retval 0 : 완료
 *         1 : isn't ready
 *         2 : sensor error
 */
uint8_t BMI323_GetData(void)
{
	uint8_t retVal = BMI323_DataReady();
	if(retVal) return retVal;

	BMI323_Get6AxisRawData();

	BMI323_ConvertGyroRaw2Dps();
	BMI323_ConvertAccRaw2G();


	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief 6축 데이터를 레지스터 레벨에서 로딩
 * @detail SCALED_IMU2에 저장.
 * @retval None
 */
void BMI323_Get6AxisRawData()
{
	uint16_t data[7] = {0,};

	BMI323_Readbytes(ACC_DATA_X, 7, &data[0]);

	msg.scaled_imu2.time_boot_ms = msg.system_time.time_boot_ms;

	msg.scaled_imu2.xacc = data[0];
	msg.scaled_imu2.yacc = data[1];
	msg.scaled_imu2.zacc = data[2];
	msg.scaled_imu2.xgyro = data[3];
	msg.scaled_imu2.ygyro = data[4];
	msg.scaled_imu2.zgyro = data[5];

	msg.scaled_imu2.temperature = data[6];

	return;
}


/*
 * @brief GYRO RAW를 mdps로 변환
 * @detail SCALED_IMU2에 저장.
 * 			m degree/s
 * @parm none
 * @retval none
 */
void BMI323_ConvertGyroRaw2Dps(void)
{
//	float sensitivity;
//
//	msg.scaled_imu.time_boot_ms = msg.system_time.time_boot_ms;
//
//	// m degree
//	msg.scaled_imu.xgyro = (float)raw_imu.xgyro / sensitivity * 1000;
//	msg.scaled_imu.ygyro = (float)raw_imu.ygyro / sensitivity * 1000;
//	msg.scaled_imu.zgyro = (float)raw_imu.zgyro / sensitivity * 1000;
//
//	return;
}


/*
 * @brief Acc RAW를 mG로 변환
 * @detail SCALED_IMU2에 저장.
 * 			mG (Gauss)
 * @parm none
 * @retval none
 */
void BMI323_ConvertAccRaw2G(void)
{
//	float sensitivity;
//
//	// mG
//	scaled_imu.xacc = (float)raw_imu.xacc / sensitivity * 1000;
//	scaled_imu.yacc = (float)raw_imu.yacc / sensitivity * 1000;
//	scaled_imu.zacc = (float)raw_imu.zacc / sensitivity * 1000;
//
//	return;
}


/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void)
{
	LL_GPIO_ResetOutputPin(GYRO2_NSS_GPIO_Port, GYRO2_NSS_Pin);
}

inline static void CHIP_DESELECT(void)
{
	LL_GPIO_SetOutputPin(GYRO2_NSS_GPIO_Port, GYRO2_NSS_Pin);
}


uint16_t BMI323_Readbyte(uint8_t reg_addr)
{
	uint16_t val=0;
	uint8_t* p = (uint8_t*)&val;

	CHIP_SELECT();

	SPI3_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	SPI3_SendByte(0x00); //Send DUMMY to read data

	p[0] = SPI3_SendByte(0x00); //Send DUMMY to read data
	p[1] = SPI3_SendByte(0x00); //Send DUMMY to read data

	CHIP_DESELECT();

	return val;
}

void BMI323_Readbytes(uint8_t reg_addr, uint8_t len, uint16_t* data)
{
	CHIP_SELECT();

	SPI3_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	SPI3_SendByte(0x00); //Send DUMMY to read data

	for(int i=0; i<len; i++)
	{
		uint8_t* p = (uint8_t*)&data[i];
		p[0] = SPI3_SendByte(0x00);
		p[1] = SPI3_SendByte(0x00);
	}

	CHIP_DESELECT();
	return;
}


void BMI323_Writebyte(uint8_t reg_addr, uint16_t val)
{
	uint8_t* p = (uint8_t*)&val;

	CHIP_SELECT();

	SPI3_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.

	SPI3_SendByte(p[0]); //Send Data to write
	SPI3_SendByte(p[1]); //Send Data to write

	CHIP_DESELECT();
}


int BMI323_Writebyte_S(uint8_t addr, uint16_t val)
{
	BMI323_Writebyte(addr, val);

	if(BMI323_Readbyte(addr) != val) return 1;

	return 0;
}


/*
 * @retval 0 : Data is ready
 *         1 : isn't ready
 *         2 : sensor error
 */
uint8_t BMI323_DataReady(void)
{
	uint16_t temp = 0;
	temp = BMI323_Readbyte(STATUS);

	// check error
	if(temp&0xFF1F) return  0x2;

	if ( ((temp>>5)&0x03) != 0x03) return 0x1;

	return 0;
}

int BMI323_initTry(uint8_t addr, uint16_t value)
{
	uint16_t temp = 0;

	uint8_t i=0;
	while(1)
	{
		temp = BMI323_Readbyte(addr);
		if((temp&0xFF) == (value&0xFF)) break;

		if(i>=5) return 1;

		HAL_Delay(1);
		i++;
	}
	delay_us(20);
	return 0;
}

// time > 20us 권장
void delay_us(uint16_t time)
{
	uint64_t previous = msg.system_time.time_unix_usec;
	while(1)
	{
		if((msg.system_time.time_unix_usec - previous) > time) break;
	}
	return;
}
