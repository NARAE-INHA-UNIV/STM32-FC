/*
 * Param_type.h
 *
 *  Created on: Mar 29, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_PARAM_PARAM_TYPE_H_
#define INC_FC_PARAM_PARAM_TYPE_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


/* Macro ---------------------------------------------------------------------*/
#define RC_CHANNEL_MAX (18)
#define SERVO_CHANNEL_MAX (12)


/* Typedef -------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint8_t version;
	uint8_t subVersion;
	uint16_t len;
} PARAM_HEADER;


/* INS -----------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	float sensitivity;
} PARAM_INS_IMU;

typedef struct __attribute__((packed)){
	PARAM_INS_IMU GYRO1;	// ICM-42688-P
	PARAM_INS_IMU ACC1;
	PARAM_INS_IMU GYRO2;	// BMI323
	PARAM_INS_IMU ACC2;
} PARAM_INS;


/* RC ------------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint16_t MIN;
	uint16_t MAX;
	uint16_t TRIM;
	uint8_t DZ;
	uint16_t OPTION;
} PARAM_RC_CH;

typedef struct __attribute__((packed)){
	uint8_t THR;
	uint8_t ROL;
	uint8_t PIT;
	uint8_t YAW;
} PARAM_RC_MAP;

typedef struct __attribute__((packed)){
	float OVERRIDE_TIME;
	uint16_t OPTIONS;
	uint16_t PROTOCOLS;
	float FS_TIMEOUT;
	uint32_t reversedMask;
	PARAM_RC_CH channel[RC_CHANNEL_MAX];
	PARAM_RC_MAP map;
} PARAM_RC;


/* SERIAL --------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint16_t baud;
	uint8_t protocol;
	uint16_t options;
} PARAM_SERIAL;


/* SERVO ---------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	uint16_t MIN;
	uint16_t MAX;
	uint16_t TRIM;
	uint8_t REVERSED : 1;
	int16_t FUNCTION;
} PARAM_SERVO_CH;

typedef struct __attribute__((packed)){
	uint8_t AUTO_TRIM : 1;
	uint16_t RATE;
	uint8_t DSHOT_RATE : 3;
	uint8_t DSHOT_ESC : 3;
	uint32_t GPIO_MASK;
	uint16_t RC_FS_MSK;
	uint8_t _32_ENABLE :1;
	PARAM_SERVO_CH channel[SERVO_CHANNEL_MAX];
} PARAM_SERVO;


/* PID -----------------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	float kp;
	float ki;
	float kd;
} pidc_gains_t;

typedef struct __attribute__((packed)){
	struct __attribute__((packed)){
		pidc_gains_t roll;
		pidc_gains_t pitch;
		pidc_gains_t yaw;
	} ANGLE;
	struct __attribute__((packed)){
		pidc_gains_t roll;
		pidc_gains_t pitch;
		pidc_gains_t yaw;
	} RATE;
} PARAM_PID;

#endif /* INC_FC_PARAM_PARAM_TYPE_H_ */
