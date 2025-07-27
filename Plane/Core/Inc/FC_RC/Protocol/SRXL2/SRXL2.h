/*
 * FC_RC/Protocol/SRXL2/driver.h
 *
 *  Created on: June 7, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_PROTOCOL_SRXL2_SRXL2_H_
#define INC_FC_RC_PROTOCOL_SRXL2_SRXL2_H_


/* Macro ---------------------------------------------------------------------*/
#define SRXL_MAX_BUFFER_SIZE    (80)

/* Functions 1 ---------------------------------------------------------------*/
int SRXL2_connect(void);
int SRXL2_getControlData(void);
uint8_t SRXL2_getRssi(void);
int SRXL2_readByteIRQ2(const uint8_t data);

#endif
