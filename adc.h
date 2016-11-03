/*
  Not part of Grbl. KeyMe specific
*/

#ifndef adc_h
#define adc_h

typedef enum {
  X = 0,
  Y,
  Z,
  C,
  FORCE,
  REV
} channel_t;

uint16_t raw_adc_readings[VOLTAGE_SENSOR_COUNT];

void adc_init();

#endif
