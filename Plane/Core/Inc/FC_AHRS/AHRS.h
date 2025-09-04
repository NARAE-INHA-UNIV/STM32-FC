/*
 * AHRS/driver.h
 *
 *  Created on: Jul 23, 2025
 *      Author: rlawn, leecurrent04
 *      Email : (rlawn)
 *      		leecurrent04@inha.edu (leecurrent04)
 */

#ifndef INC_FC_AHRS_AHRS_H_
#define INC_FC_AHRS_AHRS_H_


/* Includes ------------------------------------------------------------------*/
/* Variables -----------------------------------------------------------------*/
int AHRS_Initialization(void);
int AHRS_GetData(void);
int AHRS_CalibrateOffset(void);


/* Functions -----------------------------------------------------------------*/


#endif /* INC_FC_AHRS_AHRS_H_ */
