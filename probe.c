/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014 Sungeun K. Jeon, Adam Shelly

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
#include "probe.h"
#include "counters.h"

uint8_t probe_sense; //last known state

// Probe pin initialization routine.
void probe_init() 
{
  PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins
  PROBE_PORT |= PROBE_MASK;   // Enable internal pull-up resistors. Normal high operation.

  // Configure Timer 2: Probe debounce timer
  //  When probe signal goes high, start timer. by writing rate to TCCR2B
  //  don't count another leading edge until time has elapsed - TIFR2.ocf2a will be set.
  //  clear counts and reset when triggered.
  // Get maximum dwell out of 8 bit timer by using phase correct pwm mode, 
  //  which counts up and down; and longest prescaler.
  
  TIMSK2 &= ~((1<<OCIE2B) | (1<<OCIE2A) | (1<<TOIE2)); // Disconnect OC0 outputs and OVF interrupt.
  TCCR2A = 1; // Phase Correct PWM
  TCCR2B = (1<<CS22)|7; // 1024 prescale, use OCRA2 as top 
  OCR2A = 200;  //200*2 (for up/down) * 1024 prescale / 16Mhz = 25.6 ms
  OCR2B = 0;
  //check tifr2.


}


// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state()
{
  return(!(PROBE_PIN & PROBE_MASK));
}

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor()
{
  uint8_t probe_on = probe_get_state();
  if (sysflags.probe_state == PROBE_ACTIVE && probe_on) {
    sysflags.probe_state = PROBE_OFF;
    memcpy(sys.probe_position, sys.position, sizeof(float)*N_AXIS);
    SYS_EXEC |= EXEC_FEED_HOLD;
  }
  uint8_t change = probe_sense^probe_on;
  if (change&probe_on){
    if (TIFR2&(1<<TOV2)){ //timer has elapsed, ok to use.
      counters.counts[C_AXIS]++;
      TIFR2=(1<<TOV2); //clear ovf 
      TCNT2=0;         //restart period
    }
  }
  probe_sense=probe_on;
}


