/*
 * FC_RC/driver_SRXL2.h
 *
 *  Created on: Mar 8, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_RC_DRIVER_SRXL2_H_
#define INC_FC_RC_DRIVER_SRXL2_H_

#include <FC_RC/SRXL2_type.h>

int SRXL2_Initialization(void);
int SRXL2_Connect(void);

int SRXL2_GetData(void);
SRXL2_SignalQuality_Data SRXL2_reqSignalQuality(void);


// In Progress!
int SRXL2_SendTelemetryData(void);

#endif /* INC_FC_RC_DRIVER_SRXL2_H_ */
