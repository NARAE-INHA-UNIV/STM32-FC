/*
 * ICM42688P.c
 * FC_AHRS/FC_IMU/ICM42688P/ICM42688P.c
 *
 *  Created on: May 1, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/FC_IMU/ICM42688P/ICM42688P_module.h>


/* Variables -----------------------------------------------------------------*/
// 출력 결과를 저장하는 변수의 주소를 저장.
SCALED_IMU* icm42688p;


/* Functions -----------------------------------------------------------------*/
/*
 * @brief 초기 설정
 * @detail SPI 연결 수행, 감도 설정, offset 제거
 * @retval 0 : 완료
 * @retval 1 : 센서 없음
 */
uint8_t ICM42688P_Initialization(SCALED_IMU* p)
{
	icm42688p = p;

	SPI_Enable(DEVICE_SPI);
	CHIP_DESELECT();

	if(ICM42688P_readbyte(WHO_AM_I) != 0x47)
	{
		return 1;
	}

	// PWR_MGMT0
	ICM42688P_writebyte(PWR_MGMT0, 0x0F); // Temp on, ACC, GYRO LPF Mode
	HAL_Delay(50);

	// GYRO_CONFIG0
	ICM42688P_writebyte(GYRO_CONFIG0, 0x26); // Gyro sensitivity 1000 dps, 1kHz
	HAL_Delay(50);
	ICM42688P_writebyte(GYRO_CONFIG1, 0x00); // Gyro temp DLPF 4kHz, UI Filter 1st, 	DEC2_M2 reserved
	HAL_Delay(50);

	ICM42688P_writebyte(ACCEL_CONFIG0, 0x46); // Acc sensitivity 4g, 1kHz
	HAL_Delay(50);
	ICM42688P_writebyte(ACCEL_CONFIG1, 0x00); // Acc UI Filter 1st, 	DEC2_M2 reserved
	HAL_Delay(50);

	ICM42688P_writebyte(GYRO_ACCEL_CONFIG0, 0x11); // LPF default max(400Hz,ODR)/4
	HAL_Delay(50);

	ICM42688P_getSensitivity();

	return 0; //OK
}


/*
 * @brief 데이터 로드
 * @detail 자이로, 가속도 및 온도 데이터 로딩, 물리량 변환
 * @retval 0 : 완료
 */
uint8_t ICM42688P_GetData(void)
{
	// Check data is ready
	if(ICM42688P_dataReady()) return 1;

	ICM42688P_get6AxisRawData(&msg.raw_imu);

	icm42688p->time_boot_ms = msg.system_time.time_boot_ms;

	ICM42688P_convertGyroRaw2Dps();
	ICM42688P_convertAccRaw2G();

	return 0;
}


/*
 * @brief Offset 캘리브레이션
 * @detail
 * @param none
 * @retval none
 */
uint8_t ICM42688P_CalibrateOffset(int samples)
{
	int16_t offset[6] = {0,};

	for(int cnt=0; cnt<samples;)
	{
		RAW_IMU imu;
		if(ICM42688P_get6AxisRawData(&imu)){
			continue;
		}
		offset[0] += imu.xacc;
		offset[1] += imu.yacc;
		offset[2] += imu.zacc;
		offset[3] += imu.xgyro;
		offset[4] += imu.ygyro;
		offset[5] += imu.zgyro;

		cnt++;
	}

	offset[0] *= (-1/samples);
	offset[1] *= (-1/samples);
	offset[2] *= (-1/samples);
	offset[3] *= (-1/samples);
	offset[4] *= (-1/samples);
	offset[5] *= (-1/samples);

	ICM42688P_writebyte(OFFSET_USER0, offset[0]&0xFF);
	ICM42688P_writebyte(OFFSET_USER1, ( ((offset[0]>>8)&0xF) | ((offset[1]>>8)&0xF)<<4) );
	ICM42688P_writebyte(OFFSET_USER2, offset[1]&0xFF);
	ICM42688P_writebyte(OFFSET_USER3, offset[2]&0xFF);
	ICM42688P_writebyte(OFFSET_USER4, ( ((offset[2]>>8)&0xF) | ((offset[3]>>8)&0xF)<<4) );
	ICM42688P_writebyte(OFFSET_USER5, offset[3]&0xFF);
	ICM42688P_writebyte(OFFSET_USER6, offset[4]&0xFF);
	ICM42688P_writebyte(OFFSET_USER7, ( ((offset[4]>>8)&0xF) | ((offset[5]>>8)&0xF)<<4));
	ICM42688P_writebyte(OFFSET_USER8, offset[5]&0xFF);


	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief GYRO RAW를 m rad/s로 변환
 * @detail SCALED_IMU에 저장.
 * 			m rad/s
 * @param none
 * @retval none
 */
void ICM42688P_convertGyroRaw2Dps(void)
{
	SCALED_IMU* imu = icm42688p;
	float sensitivity = param.ins.imu0.gyro.sensitivity;

	// m degree
	imu->xgyro = (int16_t)(DEG2RAD(msg.raw_imu.xgyro/sensitivity)*1000 + 0.5f);
	imu->ygyro = (int16_t)(DEG2RAD(msg.raw_imu.ygyro/sensitivity)*1000 + 0.5f);
	imu->zgyro = (int16_t)(DEG2RAD(msg.raw_imu.zgyro/sensitivity)*1000 + 0.5f);

	return;
}



/*
 * @brief Acc RAW를 mG로 변환
 * @detail SCALED_IMU에 저장.
 * 			mG (9.8m/s^2)
 * @param none
 * @retval none
 */
void ICM42688P_convertAccRaw2G(void)
{
	SCALED_IMU* imu = icm42688p;
	float sensitivity = param.ins.imu0.acc.sensitivity;

	// mG
	imu->xacc = (int16_t)(msg.raw_imu.xacc/sensitivity * 1000 + 0.5f);
	imu->yacc = (int16_t)(msg.raw_imu.yacc/sensitivity * 1000 + 0.5f);
	imu->zacc = (int16_t)(msg.raw_imu.zacc/sensitivity * 1000 + 0.5f);

	return;
}


/*
 * @brief 6축 데이터를 레지스터 레벨에서 로딩
 * @detail RAW_IMU에 저장
 * @retval 0
 */
int ICM42688P_get6AxisRawData(RAW_IMU* imu)
{
	uint8_t data[14];

	ICM42688P_readbytes(TEMP_DATA1, sizeof(data)/sizeof(data[0]), data);

	imu->time_usec = msg.system_time.time_unix_usec;
	imu->temperature = (data[0] << 8) | data[1];
	imu->xacc = (data[2] << 8) | data[3];
	imu->yacc = (data[4] << 8) | data[5];
	imu->zacc = ((data[6] << 8) | data[7]);
	imu->xgyro = ((data[8] << 8) | data[9]);
	imu->ygyro = ((data[10] << 8) | data[11]);
	imu->zgyro = ((data[12] << 8) | data[13]);
	imu->id = 0;

	return 0;
}


/*
 * @brief 민감도 값 로드
 * @detail 레지스터로부터 로드
 * @retval 0 : 완료
 */
int ICM42688P_getSensitivity(void)
{
	float sensitivity;

	uint8_t gyro_reg_val = ICM42688P_readbyte(GYRO_CONFIG0);
	uint8_t gyro_fs_sel = (gyro_reg_val >> 5) & 0x07;

	uint8_t acc_reg_val = ICM42688P_readbyte(ACCEL_CONFIG0);
	uint8_t acc_fs_sel = (acc_reg_val >> 5) & 0x07;

	switch (gyro_fs_sel)
	{
	case 0: sensitivity = 16.4f; break;       // ±2000 dps
	case 1: sensitivity = 32.8f; break;       // ±1000 dps
	case 2: sensitivity = 65.5f; break;       // ±500 dps
	case 3: sensitivity = 131.0f; break;      // ±250 dps
	case 4: sensitivity = 262.0f; break;      // ±125 dps
	case 5: sensitivity = 524.3f; break;      // ±62.5 dps
	case 6: sensitivity = 1048.6f; break;     // ±31.25 dps
	case 7: sensitivity = 2097.2f; break;     // ±15.625 dps
	default: sensitivity = 16.4f; break;      // fallback: ±2000 dps
	}
	param.ins.imu0.gyro.sensitivity = sensitivity;

	switch (acc_fs_sel)
	{
	case 0: sensitivity = 2048.0f; break;    // ±16g
	case 1: sensitivity = 4096.0f; break;    // ±8g
	case 2: sensitivity = 8192.0f; break;    // ±4g
	case 3: sensitivity = 16384.0f; break;   // ±2g
	default: sensitivity = 2048.0f; break;   // fallback: ±16g
	}
	param.ins.imu0.acc.sensitivity = sensitivity;

	return 0;
}


/*
 * @brief 데이터가 준비되었는지 확인
 * @detail
 * 		ICM42688_
 * 		값 수신하기 전 확인
 * @retval 0 : 완료
 */
int ICM42688P_dataReady(void)
{
	uint8_t temp = 0;
	temp =ICM42688P_readbyte(INT_STATUS);

	if((temp>>3)&0x01) return 0;
	return 1;
}


/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void)
{
	LL_GPIO_ResetOutputPin(GYRO1_NSS_GPIO_Port, GYRO1_NSS_Pin);
}

inline static void CHIP_DESELECT(void)
{
	LL_GPIO_SetOutputPin(GYRO1_NSS_GPIO_Port, GYRO1_NSS_Pin);
}


uint8_t ICM42688P_readbyte(uint8_t reg_addr)
{
	uint8_t val;

	CHIP_SELECT();
	SPI_SendByte(DEVICE_SPI, reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI_SendByte(DEVICE_SPI, 0x00); //Send DUMMY to read data
	CHIP_DESELECT();
	
	return val;
}

void ICM42688P_readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;

	CHIP_SELECT();
	SPI_SendByte(DEVICE_SPI, reg_addr | 0x80); //Register. MSB 1 is read instruction.
	while(i < len)
	{
		data[i++] = SPI_SendByte(DEVICE_SPI, 0x00); //Send DUMMY to read data
	}
	CHIP_DESELECT();
}

void ICM42688P_writebyte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT();
	SPI_SendByte(DEVICE_SPI, reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI_SendByte(DEVICE_SPI, val); //Send Data to write
	CHIP_DESELECT();
}

void ICM42688P_writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;
	CHIP_SELECT();
	SPI_SendByte(DEVICE_SPI, reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	while(i < len)
	{
		SPI_SendByte(DEVICE_SPI, data[i++]); //Send Data to write
	}
	CHIP_DESELECT();
}


