/*
 * FC_Gyro/driver_ICM42688.h
 *
 *  Created on: Mar 8, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_GYRO_DRIVER_ICM42688_H_
#define INC_FC_GYRO_DRIVER_ICM42688_H_

#include "main.h"

/**
 * @brief ICM42688 structure definition.
 */

typedef struct _ICM42688{
	short temperature_raw;
	short acc_x_raw;
	short acc_y_raw;
	short acc_z_raw;
	short gyro_x_raw;
	short gyro_y_raw;
	short gyro_z_raw;

	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}Struct_ICM42688;

/**
 * @brief ICM42688 structure definition.
 */

extern Struct_ICM42688 ICM42688;
extern int32_t gyro_x_offset, gyro_y_offset, gyro_z_offset;

/**
 * @brief ICM42688 function prototype definition.
 */

int ICM42688_Initialization(void);
void ICM42688_Get6AxisRawData(void);
void ICM42688_Get3AxisGyroRawData(short* gyro);
void ICM42688_Get3AxisAccRawData(short* accel);
int ICM42688_DataReady(void);


#endif /* INC_SEN_ICM42688_DRIVER_H_ */
