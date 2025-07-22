/*
 * BMI323.h
 * FC_AHRS/FC_IMU/BMI323/BMI323.h
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_IMU_BMI323_BMI321_H_
#define INC_FC_AHRS_FC_IMU_BMI323_BMI321_H_


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <FC_AHRS/FC_IMU/BMI323/driver.h>
#include <FC_AHRS/FC_IMU/BMI323/register_map.h>
#include <GCS_MiniLink/GCS_MiniLink.h>


/* Variables -----------------------------------------------------------------*/
extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;


/* Functions 1 ---------------------------------------------------------------*/
void BMI323_Get6AxisRawData(void);
void BMI323_ConvertGyroRaw2Dps(void);
void BMI323_ConvertAccRaw2G(void);
uint8_t BMI323_DataReady(void);


/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void);
inline static void CHIP_DESELECT(void);
unsigned char SPI3_SendByte(unsigned char data);

uint16_t BMI323_Readbyte(uint8_t reg_addr);
void BMI323_Readbytes(uint8_t reg_addr, uint8_t len, uint16_t* data);
void BMI323_Writebyte(uint8_t reg_addr, uint16_t val);
int BMI323_Writebyte_S(uint8_t reg_addr, uint16_t val);

int BMI323_initTry(uint8_t addr, uint16_t value);
void delay_us(uint16_t time);


#endif /* INC_FC_AHRS_FC_IMU_BMI323_BMI321_H_ */
