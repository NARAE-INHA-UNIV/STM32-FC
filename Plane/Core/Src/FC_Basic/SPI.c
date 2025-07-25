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
/*
 * @brief SPI 활성화
 * @detail
 * 		SPI가 활성화 되어있지 않으면, 활성화함.
 * @param
 * 		SPI_TypeDef* spi : SPI 주소 (ex. SPI1)
 * @retval
 * 		None
 */
void SPI_Enable(SPI_TypeDef* spi){
	if(!LL_SPI_IsEnabled(spi)){
		LL_SPI_Enable(spi);
	}
	return;
}


/*
 * @brief SPI 1Byte 전송
 * @detail
 * 		1Byte를 전송하고 리턴된 값을 반환함.
 * 		4-wire SPI에서 동작.
 * @param
 * 		SPI_TypeDef* spi : SPI 주소 (ex. SPI1)
 *		uint8_t data : 송신할 데이터
 * @retval
 * 		uint8_t : 수신 데이터
 */
uint8_t SPI_SendByte(SPI_TypeDef* spi, uint8_t data)
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
