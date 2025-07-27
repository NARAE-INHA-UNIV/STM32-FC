/*
 * PPM.h
 *
 *  Created on: Apr 5, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_PROTOCOL_PPM_H_
#define INC_FC_RC_PROTOCOL_PPM_H_


/* Includes ------------------------------------------------------------------*/
#include <FC_RC/RC.h>
#include <main.h>

#include <FC_RC/RC_module.h>
#include <FC_Serial/MiniLink/MiniLink.h>


/* Macro ---------------------------------------------------------------------*/
#define PPM_MAX_CHANNEL 8
#define PPM_MAX_BUFFER_SIZE PPM_MAX_CHANNEL


/* Functions 1 ---------------------------------------------------------------*/
int PPM_init(void);
int PPM_readData(uint16_t data);
int PPM_getControlData(void);


#endif /* INC_FC_RC_PROTOCOL_PPM_H_ */
