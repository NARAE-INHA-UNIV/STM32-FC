/*
 * FC_Baro/driver.h
 *
 *  Created on: June 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_BARO_DRIVER_H_
#define INC_FC_BARO_DRIVER_H_


#include <stdint.h>


/* Functions -----------------------------------------------------------------*/
uint8_t Baro_Initialization(void);
unsigned int Baro_GetData(void);


#endif
