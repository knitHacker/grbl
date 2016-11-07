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
#include "protocol.h"
#include "stepper.h"
#include "planner.h"
#include "settings.h"
#include "limits.h"
#include "gcode.h"

struct probe_state probe;

// Idx, Mask, PIN - Note PIN is not the
// pin of the microcontroller, but Port In.
static struct {
  uint8_t idx;
  uint8_t mask;
  volatile uint8_t *in_port;
} sensor_map[] = {
  {
    .idx = MAG_SENSOR,
    .mask = MAGAZINE_ALIGNMENT_MASK,
    .in_port = &MAGAZINE_ALIGNMENT_PIN
  }
};

// Probe pin initialization routine.
void probe_init() 
{
  // Configure as input pins
  MAGAZINE_ALIGNMENT_DDR &= ~(MAGAZINE_ALIGNMENT_MASK);

  // Enable internal pull-up resistors. Normal high operation.
  MAGAZINE_ALIGNMENT_PORT |= MAGAZINE_ALIGNMENT_MASK;

  probe.active_sensor = E_SENSOR_TYPES;
  probe.probe_reached = 0;
  probe.isprobing = 0;

}

void probe_check()
{
  if (probe_get_active_sensor_state())
    probe.isprobing = 0;
}

void probe_plan_move(uint8_t sensor, uint8_t axis, uint8_t dir)
{
  /*  sensor - Which sensor to home
   *  axis - Which axis to move to the sensor
   *  dir - true = neg; false = pos
   */

  // Set the active probe
  probe.active_sensor = sensor;

  // Approach has all bits set (neg dir) or none (pos dir)
  const uint8_t approach = dir ? ~0 : 0;

  // Limit travel distance to the max flag
  const float travel = settings.max_travel[axis];
  const uint8_t flipped = settings.homing_dir_mask >> X_DIRECTION_BIT;  //assumes keyme configuration.
 
  // Set target for axis
  float target[N_AXIS] = {0};
  target[axis] = ((flipped & (1 << axis)) ^ approach) ? -travel : travel;

  // Set speed
  //TODO: Allow for this rate to be set over serial / passed as a variable
  const float probe_rate = settings.homing_seek_rate[axis];

  // Reset plan and stepper buffers
  plan_reset();
  st_reset();

  // Plan moition. Planner buffer should be empty, bypass mc_line() 
  plan_buffer_line(target, probe_rate, false, LINENUMBER_EMPTY_BLOCK); 

  // Tell the system we are probing
  probe.isprobing = 1;

  // Enable hard limit checking
  limits_enable(LIMIT_MASK & HARDSTOP_MASK, 0);


}

void probe_loop()
{
  // Start stepper
  st_prep_buffer();
  st_wake_up();

  while (probe.isprobing) {
    // Check for user reset and allow
    // protocol_execute_runtime to run in this loop 
    protocol_execute_runtime();
    
    if (SYS_EXEC & EXEC_RESET) {
      protocol_execute_runtime();
      return;
    } 

    if (ESTOP_PIN & ESTOP_MASK) {
      sys.alarm |= ALARM_ESTOP;
      SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
      protocol_execute_runtime();
      return;
    }

    // Check if we never reach probe.
    if ((SYS_EXEC & EXEC_CYCLE_STOP) ) {
      sys.alarm |= ALARM_PROBE_FAIL;
      SYS_EXEC |= EXEC_CRIT_EVENT;
      protocol_execute_runtime();
      return;
    }
  }

  // Disable limits
  limits_disable();

  // Force kill steppers and reset
  // step segment buffer
  st_reset();
  plan_reset(); 

}

void probe_move_to_sensor(enum e_sensor sensor, enum e_axis axis, uint8_t dir)
{
  sys.state = STATE_PROBING;
  probe_plan_move(sensor, axis, dir);
  probe_loop();

  gc_sync_position();

  sys.state = STATE_IDLE;
  st_go_idle();
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

  if (ESTOP_PIN & ESTOP_MASK) {
    sys.alarm |= ALARM_ESTOP;
    SYS_EXEC |= (EXEC_FEED_HOLD|EXEC_ALARM|EXEC_CRIT_EVENT);
  }
}

