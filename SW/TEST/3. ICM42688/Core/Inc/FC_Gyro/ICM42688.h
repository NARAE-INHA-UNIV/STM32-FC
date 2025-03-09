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

#include "main.h"

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

/**
 * @brief ICM-42688 Register Map
 */
// USER BANK 0 REGISTER
#define DEVICE_CONFIG 0x11
#define DRIVE_CONFIG 0x13
#define INT_CONFIG 0x14
#define FIFO_CONFIG 0x16
#define TEMP_DATA1 0x1D
#define TEMP_DATA0 0x1E
#define ACCEL_DATA_X1 0x1F
#define ACCEL_DATA_X0 0x20
#define ACCEL_DATA_Y1 0x21
#define ACCEL_DATA_Y0 0x22
#define ACCEL_DATA_Z1 0x23
#define ACCEL_DATA_Z0 0x24
#define GYRO_DATA_X1 0x25
#define GYRO_DATA_X0 0x26
#define GYRO_DATA_Y1 0x27
#define GYRO_DATA_Y0 0x28
#define GYRO_DATA_Z1 0x29
#define GYRO_DATA_Z0 0x2A
#define TMST_FSYNCH 0x2B
#define TMST_FSYNCL 0x2C
#define INT_STATUS 0x2D
#define FIFO_COUNTH 0x2E
#define FIFO_COUNTL 0x2F
#define FIFO_DATA 0x30
#define APEX_DATA0 0x31
#define APEX_DATA1 0x32
#define APEX_DATA2 0x33
#define APEX_DATA3 0x34
#define APEX_DATA4 0x35
#define APEX_DATA5 0x36
#define INT_STATUS2 0x37
#define INT_STATUS3 0x38
#define SIGNAL_PATH_RESET 0x4B
#define INTF_CONFIG0 0x4C
#define INTF_CONFIG1 0x4D
#define PWR_MGMT0 0x4E
#define GYRO_CONFIG0 0x4F
#define ACCEL_CONFIG0 0x50
#define GYRO_CONFIG1 0x51
#define GYRO_ACCEL_CONFIG0 0x52
#define ACCEL_CONFIG1 0x53
#define TMST_CONFIG 0x54
#define APEX_CONFIG0 0x56
#define SMD_CONFIG 0x57
#define FIFO_CONFIG1 0x5F
#define FIFO_CONFIG2 0x60
#define FIFO_CONFIG3 0x61
#define FSYNC_CONFIG 0x62
#define INT_CONFIG0 0x63
#define INT_CONFIG1 0x64
#define INT_SOURCE0 0x65
#define INT_SOURCE1 0x66
#define INT_SOURCE3 0x68
#define INT_SOURCE4 0x69
#define FIFO_LOST_PKT0 0x6C
#define FIFO_LOST_PKT1 0x6D
#define SELF_TEST_CONFIG 0x70
#define WHO_AM_I 0x75
#define REG_BANK_SEL 0x76
// USER BANK 1 REGISTER
#define SENSOR_CONFIG0 0x03
#define GYRO_CONFIG_STATIC2 0x0B
#define GYRO_CONFIG_STATIC3 0x0C
#define GYRO_CONFIG_STATIC4 0x0D
#define GYRO_CONFIG_STATIC5 0x0E
#define GYRO_CONFIG_STATIC6 0x0F
#define GYRO_CONFIG_STATIC7 0x10
#define GYRO_CONFIG_STATIC8 0x11
#define GYRO_CONFIG_STATIC9 0x12
#define GYRO_CONFIG_STATIC10 0x13
#define XG_ST_DATA 0x5F
#define YG_ST_DATA 0x60
#define ZG_ST_DATA 0x61
#define TMSTVAL0 0x62
#define TMSTVAL1 0x63
#define TMSTVAL2 0x64
#define INTF_CONFIG4 0x7A
#define INTF_CONFIG5 0x7B
#define INTF_CONFIG6 0x7C
// USER BANK 2 REGISTER
#define ACCEL_CONFIG_STATIC2 0x03
#define ACCEL_CONFIG_STATIC3 0x04
#define ACCEL_CONFIG_STATIC4 0x05
#define XA_ST_DATA 0x3B
#define YA_ST_DATA 0x3C
#define ZA_ST_DATA 0x3D
// USER BANK 3 REGISTER
#define CLKDIV 0x2A
// USER BANK 4 REGISTER
#define APEX_CONFIG1 0x40
#define APEX_CONFIG2 0x41
#define APEX_CONFIG3 0x42
#define APEX_CONFIG4 0x43
#define APEX_CONFIG5 0x44
#define APEX_CONFIG6 0x45
#define APEX_CONFIG7 0x46
#define APEX_CONFIG8 0x47
#define APEX_CONFIG9 0x48
#define ACCEL_WOM_X_THR 0x4A
#define ACCEL_WOM_Y_THR 0x4B
#define ACCEL_WOM_Z_THR 0x4C
#define INT_SOURCE6 0x4D
#define INT_SOURCE7 0x4E
#define INT_SOURCE8 0x4F
#define INT_SOURCE9 0x50
#define INT_SOURCE10 0x51
#define OFFSET_USER0 0x77
#define OFFSET_USER1 0x78
#define OFFSET_USER2 0x79
#define OFFSET_USER3 0x7A
#define OFFSET_USER4 0x7B
#define OFFSET_USER5 0x7C
#define OFFSET_USER6 0x7D
#define OFFSET_USER7 0x7E
#define OFFSET_USER8 0x7F

#endif
