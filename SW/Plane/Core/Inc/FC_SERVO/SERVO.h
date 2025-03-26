/*
 * SERVO.h
 *
 *  Created on: Mar 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_SERVO_SERVO_H_
#define INC_FC_SERVO_SERVO_H_


/* Variables -----------------------------------------------------------------*/
/* SERVO_OUTPUT_RAW (36)
 * Superseded by ACTUATOR_OUTPUT_STATUS.
 * The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages).
 * The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
*/
typedef struct __attribute__((packed)){
	uint32_t time_usec;		// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	uint8_t port;			// Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
	uint16_t servo_raw[16];	// Servo output value
} SERVO_OUTPUT_RAW;


/*
 * ACTUATOR_OUTPUT_STATUS (375)
 * The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports).
 * This message supersedes SERVO_OUTPUT_RAW.
typedef struct __attribute__((packed)){
	uint64_t time_usec;		// Timestamp (since system boot). (us)
	uint32_t active;		// Active outputs
	float actuator[16];		// Servo / motor output array values. Zero values indicate unused channels.
} ACTUATOR_OUTPUT_STATUS;
 */




#endif /* INC_FC_SERVO_SERVO_H_ */
