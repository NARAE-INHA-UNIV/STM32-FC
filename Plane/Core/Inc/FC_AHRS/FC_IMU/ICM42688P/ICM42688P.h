/*
 * ICM42688P.h
 * FC_AHRS/FC_IMU/ICM42688P/ICM42688.h
 *
 *  Created on: May 1, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_ICM42688P_ICM42688_H_
#define INC_FC_IMU_ICM42688P_ICM42688_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <FC_Basic/SPI.h>

#include <FC_AHRS/FC_IMU/ICM42688P/driver.h>
#include <FC_AHRS/FC_IMU/ICM42688P/register_map.h>

#include <FC_Serial/MiniLink/driver.h>


/* Macros --------------------------------------------------------------------*/
#define DEVICE_SPI (SPI1)


/* Variables -----------------------------------------------------------------*/
extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;


/* Functions 1 ---------------------------------------------------------------*/
void ICM42688P_convertGyroRaw2Dps(void);
void ICM42688P_convertAccRaw2G(void);
int ICM42688P_dataReady(void);
int ICM42688P_get6AxisRawData(void);
int ICM42688P_getSensitivity(void);


/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void);
inline static void CHIP_DESELECT(void);

uint8_t ICM42688P_readbyte(uint8_t reg_addr);
void ICM42688P_readbytes(unsigned char reg_addr, unsigned char len, unsigned char* data);
void ICM42688P_writebyte(uint8_t reg_addr, uint8_t val);
void ICM42688P_writebytes(unsigned char reg_addr, unsigned char len, unsigned char* data);


#endif
