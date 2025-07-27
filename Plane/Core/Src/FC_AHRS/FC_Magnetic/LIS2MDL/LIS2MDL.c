/*
 * LIS2MDL.c
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */


#include <FC_AHRS/FC_Magnetic/LIS2MDL/LIS2MDL_module.h>


/* Functions -----------------------------------------------------------------*/
/*
 * @brief 초기 설정
 * @detail SPI 연결 수행, 감도 설정, offset 제거
 * @retval 0 : 완료
 *         -1 : 센서 에러
 */
uint8_t LIS2MDL_Initialization(void)
{

	/*
	 * datasheet p.20
	 * who am i
	 */


	SPI_Enable(DEVICE_SPI);
	CHIP_DESELECT();

	// Change 3 wire SPI to 4 wire SPI
	LIS2MDL_writebyte(CFG_REG_C, 0x24);
	HAL_Delay(50);

	if(LIS2MDL_readbyte(WHO_AM_I) != 0x40) { return -1; }

	uint8_t reg = LIS2MDL_readbyte(CFG_REG_C);
	if(((reg >> 2)&0x1) == 0){
		return -1;
	}

	// self_test
	LIS2MDL_writebyte(CFG_REG_C, reg|0x02);
	HAL_Delay(50);

	// 50 Hz ODR, Continuous mode
	LIS2MDL_writebyte(CFG_REG_A, 0x0C);
	HAL_Delay(1);

	// LPF enable
	LIS2MDL_writebyte(CFG_REG_B, 0x01);
	HAL_Delay(1);

	return 0;
}


/*
 * @brief 데이터 로드
 * @detail 자이로, 가속도 및 온도 데이터 로딩, 물리량 변환
 * @retval 0 : 완료
 *         1 : isn't ready
 *         2 : sensor error
 */
uint8_t LIS2MDL_GetData(SCALED_IMU* imu)
{
	uint8_t retVal = LIS2MDL_dataReady();
	if(retVal) return retVal;

	LIS2MDL_getRawData(&msg.raw_imu);

	LIS2MDL_convertRaw2Gauss(imu);

	return 0;
}


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @retval 0 : Data is ready
 *         1 : isn't ready
 */
uint8_t LIS2MDL_dataReady(void)
{
	uint16_t temp = 0;
	temp = LIS2MDL_readbyte(STATUS_REG);

	if ((temp&0x0F) == 0x0F) return 0;

	return 1;
}


int LIS2MDL_convertRaw2Gauss(SCALED_IMU* imu)
{
	imu->xmag = msg.raw_imu.xmag * SENSITIVITY;
	imu->ymag = msg.raw_imu.ymag * SENSITIVITY;
	imu->zmag = msg.raw_imu.zmag * SENSITIVITY;

	return 0;
}


int LIS2MDL_getRawData(RAW_IMU* imu)
{
	uint8_t data[6];

	LIS2MDL_readbytes(OUTX_L_REG, sizeof(data)/sizeof(data[0]), data);

	imu->xmag = (data[1] << 8) | data[0];
	imu->ymag = (data[3] << 8) | data[2];
	imu->zmag = ((data[5] << 8) | data[4]);

	return 0;
}

/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void)
{
	LL_GPIO_ResetOutputPin(MAG_NSS_GPIO_Port, MAG_NSS_Pin);
}

inline static void CHIP_DESELECT(void)
{
	LL_GPIO_SetOutputPin(MAG_NSS_GPIO_Port, MAG_NSS_Pin);
}


uint8_t LIS2MDL_readbyte(uint8_t reg_addr)
{
	uint8_t p = 0;

	CHIP_SELECT();
	SPI_SendByte(DEVICE_SPI, reg_addr | 0x80); //Register. MSB 1 is read instruction.
	p = SPI_SendByte(DEVICE_SPI, 0x00); //Send DUMMY to read data
	CHIP_DESELECT();

	return p;
}

void LIS2MDL_readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	CHIP_SELECT();

	SPI_SendByte(DEVICE_SPI, reg_addr | 0x80); //Register. MSB 1 is read instruction.

	for(int i=0; i<len; i++)
	{
		data[i] = SPI_SendByte(DEVICE_SPI, 0x00);
	}

	CHIP_DESELECT();
	return;
}


void LIS2MDL_writebyte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT();

	SPI_SendByte(DEVICE_SPI, reg_addr & 0x7F); 	//Register. MSB 0 is write instruction.

	SPI_SendByte(DEVICE_SPI, val); //Send Data to write

	CHIP_DESELECT();
}


