/*
 * BMI323.h
 * FC_AHRS/FC_IMU/BMI323/BMI323.h
 *
 *  Created on: June 19, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_IMU_BMI323_BMI323_MODULE_H_
#define INC_FC_AHRS_FC_IMU_BMI323_BMI323_MODULE_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <FC_Basic/SPI.h>

#include <FC_AHRS/FC_IMU/BMI323/BMI323.h>
#include <FC_AHRS/FC_IMU/BMI323/register_map.h>

#include <FC_Serial/MiniLink/MiniLink_module.h>


/* Macros --------------------------------------------------------------------*/
#define DEVICE_SPI (SPI3)


/* Variables -----------------------------------------------------------------*/


/* Functions 1 ---------------------------------------------------------------*/
uint16_t BMI323_checkNBit(uint8_t addr, uint8_t n);
uint8_t BMI323_dataReady(void);
void BMI323_convertGyroRaw2Dps(void);
void BMI323_convertAccRaw2G(void);
void BMI323_get6AxisRawData(void);


/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void);
inline static void CHIP_DESELECT(void);

uint16_t BMI323_readbyte(uint8_t reg_addr);
void BMI323_readbytes(uint8_t reg_addr, uint8_t len, uint16_t* data);
void BMI323_writebyte(uint8_t reg_addr, uint16_t val);


#endif /* INC_FC_AHRS_FC_IMU_BMI323_BMI321_H_ */
