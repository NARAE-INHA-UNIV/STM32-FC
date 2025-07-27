/*
 * LED.h
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_LED_LED_H_
#define INC_FC_BASIC_LED_LED_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Functions -----------------------------------------------------------------*/
void LED_Update(void);

void LED_SetRed(uint8_t state);
void LED_SetYellow(uint8_t state);
void LED_SetBlue(uint8_t state);

void LED_ResetRed(void);
void LED_ResetYellow(void);
void LED_ResetBlue(void);


#endif /* INC_FC_BASIC_LED_LED_H_ */
