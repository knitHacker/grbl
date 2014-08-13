/*
  counters.c - code pertaining to encoders and other counting methods
  Part of Grbl

  Copyright (c) 2014 Adam Shelly

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
  
#include "system.h"
#include "counters.h"


counters_t counters = {{0}};

// Counters pin initialization routine.
void counters_init() 
{
  //encoders and feedback //TODO: move to new file
#ifdef KEYME_BOARD
  FDBK_DDR &= ~(FDBK_MASK); // Configure as input pins
  FDBK_PORT |= FDBK_MASK;   // Enable internal pull-up resistors. Normal high operation. //TODO test
#endif
  counters.state = FDBK_PIN&FDBK_MASK; //record initial state

  FDBK_PCMSK |= FDBK_MASK;  // Enable specific pins of the Pin Change Interrupt
  PCICR |= (1 << FDBK_INT);   // Enable Pin Change Interrupt

}

// Resets the counts for an axis
void  counters_reset(uint8_t axis)
{
  counters.counts[axis]=0;
}


// Returns the counters pin state. Triggered = true.  and counters state monitor.
uint16_t counters_get_count(uint8_t axis)
{
  return counters.counts[axis];
}

uint8_t counters_get_state(){
  return counters.state;
}


// Monitors counters pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void counters_state_monitor()
{

  
}

ISR(FDBK_INT_vect) {
  uint8_t state =  FDBK_PIN&FDBK_MASK;
  uint8_t change = (state^counters.state);
  if (change & ((1<<Z_ENC_CHA_BIT)|(1<<Z_ENC_CHB_BIT))) { //if a or b changed
    counters.anew = (state>>Z_ENC_CHA_BIT)&1;
    counters.dir = counters.anew^counters.bold ? 1 : -1;
    counters.bold = (state>>Z_ENC_CHB_BIT)&1;
    counters.counts[Z_AXIS] += counters.dir;
  }
  if (change & (1<<Z_ENC_IDX_BIT)) { //idx changed
    counters.idx += counters.dir * ((state>>Z_ENC_IDX_BIT)&1);
    //TODO make sure counts are sane
  }
  if (change & (1<<MAG_SENSE_BIT)) { //mag changed
    counters.mags = (state>>MAG_SENSE_BIT)&1;
    counters.counts[C_AXIS] += counters.mags;
  }
  counters.state = state;
}
