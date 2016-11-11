/*
  Not part of Grbl. Written by KeyMe.
*/

#include "system.h"
#include "settings.h"
#include "magazine.h"
#include "limits.h"
#include "protocol.h"
#include "stepper.h"
#include "report.h"

enum magazine_edge_type {
  E_MAGAZINE_EDGE_TYPE_RISING = 0,
  E_MAGAZINE_EDGE_TYPE_FALLING
};

struct probe_window {
  int32_t inboard;
  int32_t outboard;
  bool direction;
};

static struct {
  int32_t delta_pos_limit;
  int32_t lash;
  int32_t spacing;
  struct probe_window probe_events[2];
  uint8_t evt_no;
  bool on_probe;
  uint32_t mag_events;
} mag_state;


void magazine_init()
{
  // Set the magazine alignment position to the current position
  memcpy(sys.probe_position, sys.position, sizeof(int32_t) * N_AXIS);

  mag_state.delta_pos_limit = settings.mag_gap_limit * settings.steps_per_mm[C_AXIS];
}

int32_t magazine_lash(void)
{
  return mag_state.lash;
}

int32_t magazine_spacing(void)
{
  return mag_state.spacing;
}

bool magazine_direction(void)
{
  // NOTE: the !! here is a binary clamp
  return !!(DIRECTION_PORT & get_direction_mask(C_AXIS));
}

bool magazine_lash_possible(void)
{
  // Magazine lash is possible when the carousel has changed direction
  // since the last magazine flag.
  const int last_evt_no = mag_state.evt_no ? 0 : 1;
  const struct probe_window *current_event = &mag_state.probe_events[mag_state.evt_no];
  const struct probe_window *last_event = &mag_state.probe_events[last_evt_no];

  if (mag_state.mag_events < 2) {
    return false;
  }

  return current_event->direction != last_event->direction;
}

void magazine_calculate_loss(void)
{
  const int last_evt_no = mag_state.evt_no ? 0 : 1;
  const struct probe_window *current_event = &mag_state.probe_events[mag_state.evt_no];
  const struct probe_window *last_event = &mag_state.probe_events[last_evt_no];

  // Select which slop type we can detect based on whether or not lash
  // is possible
  int32_t * const property = magazine_lash_possible() ?
    &mag_state.lash : &mag_state.spacing;

  // Which edge we use as a reference point for our slop calculation
  // depends on which direction we are moving in.
  if (magazine_direction()) {
    *property = abs(current_event->outboard - last_event->outboard);
  } else {
    *property = abs(current_event->inboard - last_event->inboard);
  }
}

void magazine_record_edge(enum magazine_edge_type edge)
{
  // Update our edge events based on which direction we are travelling
  // in, and whether or not it is a rising or falling edge
  struct probe_window *current_event = &mag_state.probe_events[mag_state.evt_no];
  if (magazine_direction() ^ (E_MAGAZINE_EDGE_TYPE_RISING == edge)) {
    current_event->inboard = sys.position[C_AXIS];
  } else {
    current_event->outboard = sys.position[C_AXIS];
  }
}

void magazine_loss_detector(const uint8_t magazine_alignment_on)
{
  // When the probe is detected, copy the current carousel position
  // into the probe position. If we were not previously on a flag,
  // begin recording a magazine flag event, otherwise, end the event
  // and calculate the loss
  if (magazine_alignment_on && !mag_state.on_probe) {
    mag_state.on_probe = true;
    mag_state.probe_events[mag_state.evt_no].direction = magazine_direction();
    magazine_record_edge(E_MAGAZINE_EDGE_TYPE_RISING);
    mag_state.mag_events++;
  } else if (mag_state.on_probe) {
    mag_state.on_probe = false;
    magazine_record_edge(E_MAGAZINE_EDGE_TYPE_FALLING);
    mag_state.evt_no = mag_state.evt_no ? 0 : 1;

    magazine_calculate_loss();
    request_report(REQUEST_SLOP_REPORT, 0);
  }

}

// Monitors the gap in units between mags and throws an alarm if the gap is larger than a
// specified threshold. Additionally, magazine slop can be calculated
// if turned on via compiler flag
void magazine_gap_monitor()
{
  const uint8_t magazine_alignment_on = magazine_get_state();
  // When the probe is detected, copy the current carousel position
  // into the probe position
  if (magazine_alignment_on) {
    sys.probe_position[C_AXIS] = sys.position[C_AXIS];
  }

#ifdef USE_CAROUSEL_LOSS
  magazine_loss_detector(magazine_alignment_on);
#endif

  if (limits.mag_gap_check == 0) {
    return;
  }

  // Activate alarm if the gap between the current position and the previous
  // probe position becomes too large. Only do this when the system has already homed
  const int32_t cur_pos = sys.position[C_AXIS];
  const int32_t probe_pos = sys.probe_position[C_AXIS];
  const int32_t delta_pos = abs(cur_pos - probe_pos);

  if (delta_pos > mag_state.delta_pos_limit) {
    sys.state = STATE_ALARM;
    sys.alarm |= ALARM_MAG_MISSING;
    SYS_EXEC |= (EXEC_FEED_HOLD | EXEC_ALARM | EXEC_CRIT_EVENT);
    st_go_idle();
    protocol_execute_runtime();
  }

}
