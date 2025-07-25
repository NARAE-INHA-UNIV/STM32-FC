/*
 * SPI.c
 *
 *  Created on: Jul 25, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_Basic/SPI.h>



/* Functions -----------------------------------------------------------------*/
void SPI_Enable(SPI_TypeDef* spi){
	if(!LL_SPI_IsEnabled(spi)){
		LL_SPI_Enable(spi);
	}
	return;
}


unsigned char SPI_SendByte(SPI_TypeDef* spi, unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(spi)==RESET);
	LL_SPI_TransmitData8(spi, data);

	while(LL_SPI_IsActiveFlag_RXNE(spi)==RESET);
	return LL_SPI_ReceiveData8(spi);
}


unsigned char SPI1_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(SPI1)==RESET);
	LL_SPI_TransmitData8(SPI1, data);

	while(LL_SPI_IsActiveFlag_RXNE(SPI1)==RESET);
	return LL_SPI_ReceiveData8(SPI1);
}


unsigned char SPI2_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(SPI2)==RESET);
	LL_SPI_TransmitData8(SPI2, data);

	while(LL_SPI_IsActiveFlag_RXNE(SPI2)==RESET);
	return LL_SPI_ReceiveData8(SPI2);
}


unsigned char SPI3_SendByte(unsigned char data)
{
	while(LL_SPI_IsActiveFlag_TXE(SPI3)==RESET);
	LL_SPI_TransmitData8(SPI3, data);

	while(LL_SPI_IsActiveFlag_RXNE(SPI3)==RESET);
	return LL_SPI_ReceiveData8(SPI3);
}
