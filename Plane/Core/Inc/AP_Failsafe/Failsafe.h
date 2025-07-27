/*
 * driver.h
 *
 *  Created on: Jul 23, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_AP_FAILSAFE_FAILSAFE_H_
#define INC_AP_FAILSAFE_FAILSAFE_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Variables -----------------------------------------------------------------*/
extern uint8_t fsFlag;


/* Functions -----------------------------------------------------------------*/
int FS_IsFailsafe(void);



#endif /* INC_AP_FAILSAFE_FAILSAFE_H_ */
