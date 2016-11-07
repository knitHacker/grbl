/*
  probe.h - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014 Sungeun K. Jeon

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

#ifndef probe_h
#define probe_h 

#define N_SENSORS 1
// Values that define the probing state machine.  
#define PROBE_OFF     0 // No probing. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.

enum e_sensor {
  MAG_SENSOR = 0,
  KEY_SENSOR,
  E_SENSOR_TYPES
};

struct probe_state {
  enum e_sensor active_sensor;  // The currently active probe seonsor
  uint8_t probe_reached;  // Flag to indicate if active probe is reached
  uint8_t isprobing;

};

extern struct probe_state probe;

// Probe pin initialization routine.
void probe_init();

// Plan a probe move to a probe sensor on an axis in a given direction
void probe_move_to_sensor(enum e_sensor, enum e_axis, uint8_t dir);

// Called from stepper ISR - needs to be very efficient
void probe_check();

// Returns active probe state
#define ACTIVE_SENSOR_MASK (sensor_map[probe.active_sensor].mask)
#define ACTIVE_SENSOR_PIN (*sensor_map[probe.active_sensor].in_port)
#define probe_get_active_sensor_state() (!(ACTIVE_SENSOR_MASK & ACTIVE_SENSOR_PIN))

// Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
#define probe_get_state() (!(PROBE_PIN & PROBE_MASK))

// Monitors probe pin state and records the system position
// when detected. Called by the stepper ISR each ISR tick.
void probe_state_monitor();

#endif
