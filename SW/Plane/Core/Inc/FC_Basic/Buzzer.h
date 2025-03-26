/*
 * FC_Basic/Buzzer.h
 *
 *  Created on: Feb 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_BUZZER_H_
#define INC_FC_BASIC_BUZZER_H_

#include <FC_Basic/driver_Buzzer.h>

#define APB1_CLOCKS 84000000L

const double tones[] = { 261.6256, 293.6648, 329.6276, 349.2282, 391.9954, 440, 493.8833, 523.2511 };

typedef enum {
    DO = 0,
    RE,
    MI,
    FA,
    SOL,
    LA,
    SI,
    DO_HIGH
} Note;

#endif /* INC_FC_BUZZER_H_ */
