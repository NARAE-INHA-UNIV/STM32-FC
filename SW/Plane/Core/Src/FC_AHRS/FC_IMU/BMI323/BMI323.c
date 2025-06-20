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
 *         1 : 센서 없음
 *         2 : 센서 없음
 *         3 :
 */
int BMI323_Initialization(void)
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
	temp = BMI323_Readbyte(CHIP_ID);
	if((temp&0xFF) != 0x43) return 1;

	// Checking the correct initialization status
	temp = BMI323_Readbyte(ERR_REG);
	if((temp&0xFF) != 0x00) return 2;

	for(int i=0; i<5; i++)
	{
		temp = BMI323_Readbyte(STATUS);
		if((temp&0xFF) == 0x01) break;
		if(i>=5) return 3;
	}


	// Configure the device for normal power mode
	BMI323_Writebyte(ACC_CONF, 0x4027);
	HAL_Delay(50);

	BMI323_Writebyte(GYR_CONF, 0x404B);
	HAL_Delay(50);

	// Enable Interrupts when data is ready

	// Remove Gyro X offset
	return 0; //OK
}


/*
 * @brief 데이터 로드
 * @detail 자이로, 가속도 및 온도 데이터 로딩, 물리량 변환
 * @retval 0 : 완료
 */
int BMI323_GetData(void)
{
	if(BMI323_DataReady()) return -1;

	BMI323_Get6AxisRawData();

	BMI323_ConvertGyroRaw2Dps();
	BMI323_ConvertAccRaw2G();

	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief 6축 데이터를 레지스터 레벨에서 로딩
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
 * @detail SCALED_IMU에 저장.
 * 			m degree/s
 * @parm none
 * @retval none
 */
void BMI323_ConvertGyroRaw2Dps(void)
{
//	uint16_t gyro_reg_val = BMI323_Readbyte(GYR_CONF);
//	uint8_t gyro_fs_sel = (gyro_reg_val >> 4) & 0x07;
//
//	float sensitivity;
//
//	switch (gyro_fs_sel)
//	{
//	case 0: sensitivity = 16.4f; break;       // ±2000 dps
//	case 1: sensitivity = 32.8f; break;       // ±1000 dps
//	case 2: sensitivity = 65.5f; break;       // ±500 dps
//	case 3: sensitivity = 131.0f; break;      // ±250 dps
//	case 4: sensitivity = 262.0f; break;      // ±125 dps
//	case 5: sensitivity = 524.3f; break;      // ±62.5 dps
//	case 6: sensitivity = 1048.6f; break;     // ±31.25 dps
//	case 7: sensitivity = 2097.2f; break;     // ±15.625 dps
//	default: sensitivity = 16.4f; break;      // fallback: ±2000 dps
//	}
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
 * @detail SCALED_IMU에 저장.
 * 			mG (Gauss)
 * @parm none
 * @retval none
 */
void BMI323_ConvertAccRaw2G(void)
{
//	uint8_t acc_reg_val = BMI323_Readbyte(ACC_CONF);
//	uint8_t acc_fs_sel = (acc_reg_val >> 4) & 0x07;
//
//	float sensitivity;
//
//	switch (acc_fs_sel)
//	{
//	case 0: sensitivity = 2048.0f; break;    // ±16g
//	case 1: sensitivity = 4096.0f; break;    // ±8g
//	case 2: sensitivity = 8192.0f; break;    // ±4g
//	case 3: sensitivity = 16384.0f; break;   // ±2g
//	default: sensitivity = 2048.0f; break;   // fallback: ±16g
//	}
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


unsigned char SPI3_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(SPI3)==RESET);
	LL_SPI_TransmitData8(SPI3, data);

	while(LL_SPI_IsActiveFlag_RXNE(SPI3)==RESET);
	return LL_SPI_ReceiveData8(SPI3);
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


int BMI323_DataReady(void)
{
	uint16_t temp = 0;
	temp = BMI323_Readbyte(STATUS);

	if ( ((temp>>5)&0x03) != 0x03) return 1;

	return 0;
}
