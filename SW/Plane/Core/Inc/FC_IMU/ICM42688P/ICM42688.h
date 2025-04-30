/**

 * FC_Gyro/ICM42688.c
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

#ifndef INC_FC_GYRO_ICM42688_H_
#define INC_FC_GYRO_ICM42688_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <GCS_MAVLink/GCS_Common.h>

#include <FC_IMU/ICM42688P/driver.h>
#include <FC_IMU/ICM42688P/register_map.h>


/* Variables -----------------------------------------------------------------*/
extern RAW_IMU raw_imu;
extern SCALED_IMU scaled_imu;

extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;


/* Macros --------------------------------------------------------------------*/
/*
ICM-42688 SPI Operational Features
1. Data is delivered MSB first and LSB last
2. Data is latched on the rising edge of SPC
3. Data should be transitioned on the falling edge of SPC
4. The maximum frequency of SPC is 10MHz
5. SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The first byte contains the
SPI Address, and the following byte(s) contain(s) the SPI data. The first bit of the first byte contains the Read/Write bit
and indicates the Read (1) or Write (0) operation. The following 7 bits contain the Register Address. In cases of multiple byte Read/Writes, data is two or more bytes:
*/

/**
 * @brief Definition for connected to SPI1 (APB2 PCLK = 168MHz)
 */
#define ICM42688_SPI_CHANNEL		SPI1

#define ICM42688_SPI_SCLK_PIN		LL_GPIO_PIN_5
#define ICM42688_SPI_SCLK_PORT		GPIOA

#define ICM42688_SPI_MISO_PIN		LL_GPIO_PIN_6
#define ICM42688_SPI_MISO_PORT		GPIOA

#define ICM42688_SPI_MOSI_PIN		LL_GPIO_PIN_7
#define ICM42688_SPI_MOSI_PORT		GPIOA

#define ICM42688_SPI_CS_PIN			LL_GPIO_PIN_5
#define ICM42688_SPI_CS_PORT		GPIOE

//#define ICM42688_INT_PIN			LL_GPIO_PIN_5
//#define ICM42688_INT_PORT			GPIOC
//#define ICM42688_INT_CLK			LL_AHB1_GRP1_PERIPH_GPIOC

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
#define CHIP_SELECT(ICM42688)		LL_GPIO_ResetOutputPin(ICM42688_SPI_CS_PORT, ICM42688_SPI_CS_PIN)
#define CHIP_DESELECT(ICM42688)		LL_GPIO_SetOutputPin(ICM42688_SPI_CS_PORT, ICM42688_SPI_CS_PIN)
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/* Functions 1 ---------------------------------------------------------------*/
void Get6AxisRawData(void);
void ConvertGyroRaw2Dps(void);
void ConvertAccRaw2G(void);
//void ICM42688_Get3AxisGyroRawData(short* gyro);
//void ICM42688_Get3AxisAccRawData(short* accel);
//int ICM42688_DataReady(void);


/* Functions 2 ---------------------------------------------------------------*/
void ICM42688_GPIO_SPI_Initialization(void);
unsigned char SPI1_SendByte(unsigned char data);
uint8_t ICM42688_Readbyte(uint8_t reg_addr);
void ICM42688_Readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data);
void ICM42688_Writebyte(uint8_t reg_addr, uint8_t val);
void ICM42688_Writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data);

#endif
