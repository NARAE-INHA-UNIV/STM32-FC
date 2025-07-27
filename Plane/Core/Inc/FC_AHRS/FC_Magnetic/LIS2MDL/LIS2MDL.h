/*
 * LIS2MDL.h
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_AHRS_FC_MAGNETIC_LIS2MDL_LIS2MDL_H_
#define INC_FC_AHRS_FC_MAGNETIC_LIS2MDL_LIS2MDL_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include <FC_Basic/SPI.h>

#include <FC_AHRS/FC_Magnetic/LIS2MDL/driver.h>
#include <FC_AHRS/FC_Magnetic/LIS2MDL/register_map.h>

#include <FC_Serial/MiniLink/MiniLink.h>


/* Macros --------------------------------------------------------------------*/
#define DEVICE_SPI (SPI3)


/* Variables -----------------------------------------------------------------*/


/* Functions 1 ---------------------------------------------------------------*/
void LIS2MDL_getRawData(void);
uint8_t LIS2MDL_dataReady(void);


/* Functions 2 ---------------------------------------------------------------*/
inline static void CHIP_SELECT(void);
inline static void CHIP_DESELECT(void);

uint8_t LIS2MDL_readbyte(uint8_t reg_addr);
void LIS2MDL_readbytes(uint8_t reg_addr, uint8_t len, uint8_t* data);
void LIS2MDL_writebyte(uint8_t reg_addr, uint8_t val);


#endif /* INC_FC_AHRS_FC_MAGNETIC_LIS2MDL_LIS2MDL_H_ */
