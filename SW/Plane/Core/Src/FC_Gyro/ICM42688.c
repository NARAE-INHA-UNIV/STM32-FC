/**

 * ICM42688.c
 * @author ChrisP @ M-HIVE

 * This library source code has been created for STM32F4. Only supports SPI.
 *
 * Development environment specifics:
 * STM32CubeIDE 1.0.0
 * STM32CubeF4 FW V1.24.1
 * STM32F4 LL Driver(SPI) and HAL Driver(RCC for HAL_Delay() function)
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2019
 * Rev. 1.0
 *
 * https://github.com/ChrisWonyeobPark/
 * https://www.udemy.com/course/stm32-drone-programming/?referralCode=E24CB7B1CD9993855D45
 * https://www.inflearn.com/course/stm32cubelde-stm32f4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C
*/

/**
 * @brief ICM42688 structure definition.
 */

#include <FC_Gyro/ICM42688.h>
#include <FC_Gyro/driver_ICM42688.h>

int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset; // To remove offset


void ICM42688_GPIO_SPI_Initialization(void)
{
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
	/**SPI1 GPIO Configuration
	PA5   ------> SPI1_SCK
	PA6   ------> SPI1_MISO
	PA7   ------> SPI1_MOSI
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8; //ICM-42688 MAX SPI CLK is 10MHz. But DIV2(42MHz) is available.
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	SPI_InitStruct.CRCPoly = 10;
	LL_SPI_Init(ICM42688_SPI_CHANNEL, &SPI_InitStruct);
	LL_SPI_SetStandard(ICM42688_SPI_CHANNEL, LL_SPI_PROTOCOL_MOTOROLA);
	
	/**ICM42688 GPIO Control Configuration
	 * PC4  ------> ICM42688_SPI_CS_PIN (output)
	 * PC5  ------> ICM42688_INT_PIN (input)
	 */
	/**/
	LL_GPIO_ResetOutputPin(ICM42688_SPI_CS_PORT, ICM42688_SPI_CS_PIN);
	
	/**/
	GPIO_InitStruct.Pin = ICM42688_SPI_CS_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ICM42688_SPI_CS_PORT, &GPIO_InitStruct);
	
	/**/
//	GPIO_InitStruct.Pin = ICM42688_INT_PIN;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
//	LL_GPIO_Init(ICM42688_INT_PORT, &GPIO_InitStruct);

	LL_SPI_Enable(ICM42688_SPI_CHANNEL);

	CHIP_DESELECT(ICM42688);
}


unsigned char SPI1_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(ICM42688_SPI_CHANNEL)==RESET);
	LL_SPI_TransmitData8(ICM42688_SPI_CHANNEL, data);
	
	while(LL_SPI_IsActiveFlag_RXNE(ICM42688_SPI_CHANNEL)==RESET);
	return LL_SPI_ReceiveData8(ICM42688_SPI_CHANNEL);
}

//////////////////////////////////////////////////////////////

uint8_t ICM42688_Readbyte(uint8_t reg_addr)
{
	uint8_t val;

	CHIP_SELECT(ICM42688);
	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	val = SPI1_SendByte(0x00); //Send DUMMY to read data
	CHIP_DESELECT(ICM42688);
	
	return val;
}

void ICM42688_Readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;

	CHIP_SELECT(ICM42688);
	SPI1_SendByte(reg_addr | 0x80); //Register. MSB 1 is read instruction.
	while(i < len)
	{
		data[i++] = SPI1_SendByte(0x00); //Send DUMMY to read data
	}
	CHIP_DESELECT(ICM42688);
}

void ICM42688_Writebyte(uint8_t reg_addr, uint8_t val)
{
	CHIP_SELECT(ICM42688);
	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	SPI1_SendByte(val); //Send Data to write
	CHIP_DESELECT(ICM42688);
}

void ICM42688_Writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data)
{
	unsigned int i = 0;
	CHIP_SELECT(ICM42688);
	SPI1_SendByte(reg_addr & 0x7F); //Register. MSB 0 is write instruction.
	while(i < len)
	{
		SPI1_SendByte(data[i++]); //Send Data to write
	}
	CHIP_DESELECT(ICM42688);
}


int ICM42688_Initialization(void)
{

	uint8_t who_am_i = 0;
	int16_t accel_raw_data[3] = {0};  // To remove offset
	int16_t gyro_raw_data[3] = {0};   // To remove offset
	
	ICM42688_GPIO_SPI_Initialization();
	
	// printf("Checking ICM42688...\n\r");
	
	who_am_i = ICM42688_Readbyte(WHO_AM_I); 

	if(who_am_i == 0x47)
	{
		// printf("ICM42688 who_am_i = 0x%02x...OK\n\r", who_am_i);
	}
	// recheck
	else if(who_am_i != 0x47)
	{
		who_am_i = ICM42688_Readbyte(WHO_AM_I); // check again WHO_AM_I (0x75)

		if (who_am_i != 0x12){
			// printf( "ICM42688 Not OK: 0x%02x Should be 0x%02x\n\r", who_am_i, 0x12);
			return 1; //ERROR
		}
	}
	
	// PWR_MGMT0
	ICM42688_Writebyte(PWR_MGMT0, 0x0F); // Temp on, ACC, GYRO LPF Mode
	HAL_Delay(50);

	
	// GYRO_CONFIG0
	ICM42688_Writebyte(GYRO_CONFIG0, 0x06); // Gyro sensitivity 2000 dps, 1kHz
	HAL_Delay(50);
	ICM42688_Writebyte(GYRO_CONFIG1, 0x00); // Gyro temp DLPF 4kHz, UI Filter 1st, 	DEC2_M2 reserved
	HAL_Delay(50);

	ICM42688_Writebyte(ACCEL_CONFIG0, 0x06); // Acc sensitivity 16g, 1kHz
	HAL_Delay(50);
	ICM42688_Writebyte(ACCEL_CONFIG1, 0x00); // Acc UI Filter 1st, 	DEC2_M2 reserved
	HAL_Delay(50);

	ICM42688_Writebyte(GYRO_ACCEL_CONFIG0, 0x11); // LPF default max(400Hz,ODR)/4
	HAL_Delay(50);

	// Enable Interrupts when data is ready
//	ICM42688_Writebyte(INT_ENABLE, 0x01); // Enable DRDY Interrupt
//	HAL_Delay(50);
	
	
	// Remove Gyro X offset
	return 0; //OK
}

void ICM42688_Get6AxisRawData()
{
	uint8_t data[14];

	ICM42688_Readbytes(TEMP_DATA1, 14, data);
	
	raw_imu.time_usec = system_time.time_unix_usec;
	raw_imu.temperature = (data[0] << 8) | data[1];
	raw_imu.xacc = (data[2] << 8) | data[3];
	raw_imu.yacc = (data[4] << 8) | data[5];
	raw_imu.zacc = ((data[6] << 8) | data[7]);
	raw_imu.xgyro = ((data[8] << 8) | data[9]);
	raw_imu.ygyro = ((data[10] << 8) | data[11]);
	raw_imu.zgyro = ((data[12] << 8) | data[13]);

	return;
}
/*

void ICM42688_Get3AxisGyroRawData(short* gyro)
{
	unsigned char data[6];
	ICM42688_Readbytes(GYRO_XOUT_H, 6, data);
	
	gyro[0] = ((data[0] << 8) | data[1]);
	gyro[1] = ((data[2] << 8) | data[3]);
	gyro[2] = ((data[4] << 8) | data[5]);
}

void ICM42688_Get3AxisAccRawData(short* accel)
{
	unsigned char data[6];
	ICM42688_Readbytes(ACCEL_XOUT_H, 6, data);
	
	accel[0] = ((data[0] << 8) | data[1]);
	accel[1] = ((data[2] << 8) | data[3]);
	accel[2] = ((data[4] << 8) | data[5]);
}

int ICM42688_DataReady(void)
{
	return LL_GPIO_IsInputPinSet(ICM42688_INT_PORT, ICM42688_INT_PIN);
}

*/
