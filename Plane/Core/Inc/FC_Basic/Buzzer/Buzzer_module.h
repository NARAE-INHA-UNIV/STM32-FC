/*
 * FC_Basic/Buzzer/Buzzer.h
 *
 *  Created on: Feb 26, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 */

#ifndef INC_FC_BASIC_BUZZER_H_
#define INC_FC_BASIC_BUZZER_H_


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <FC_Basic/Buzzer/Buzzer.h>


/* Macro ---------------------------------------------------------------------*/
#define APB1_CLOCKS 84000000L


/* Variables -----------------------------------------------------------------*/
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


/* Functions -----------------------------------------------------------------*/
void playNote(Note note, uint16_t time);


#endif /* INC_FC_BUZZER_H_ */
