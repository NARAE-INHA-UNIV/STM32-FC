/*
 * LED/driver.h
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email: leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_LED_DRIVER_H_
#define INC_FC_BASIC_LED_DRIVER_H_


/* Includes ------------------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/
void LED_SetAHRS(uint8_t state);
void LED_SetRadioControlIMU(uint8_t state);
void LED_Update(void);



#endif /* INC_FC_BASIC_LED_DRIVER_H_ */
