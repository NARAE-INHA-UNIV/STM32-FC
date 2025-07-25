/*
 * SPI.h
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_SPI_H_
#define INC_FC_BASIC_SPI_H_


/* Includes ------------------------------------------------------------------*/
#include <main.h>


/* Functions -----------------------------------------------------------------*/
void SPI_Enable(SPI_TypeDef* spi);

uint8_t SPI_SendByte(SPI_TypeDef* spi, uint8_t data);
unsigned char SPI1_SendByte(unsigned char data);
unsigned char SPI2_SendByte(unsigned char data);
unsigned char SPI3_SendByte(unsigned char data);



#endif /* INC_FC_BASIC_SPI_H_ */
