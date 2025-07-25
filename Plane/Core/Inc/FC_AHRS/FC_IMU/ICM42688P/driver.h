/*
 * ICM42688P/driver.h
 *
 *  Created on: Mar 8, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_IMU_ICM42688P_DRIVER_H_
#define INC_FC_IMU_ICM42688P_DRIVER_H_


/* Functions -----------------------------------------------------------------*/
uint8_t ICM42688_Initialization(void);
uint8_t ICM42688_GetData(void);
uint8_t ICM42688_CalibrateOffset(void);


#endif /* INC_SEN_ICM42688_DRIVER_H_ */
