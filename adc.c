
#include "system.h"
#include "adc.h"
#include "nuts_bolts.h"

#define ADMUX_SELECTION_MASK    0x7
#define MUX5_MASK               0x8

void adc_init()
{

  // These pins are initialized here because there are 
  // no other where the pins can be initialized with the
  // associated peripherals, such as with the stepper motors.
  
  #ifdef USE_LOAD_CELL
    LC_DDR &= ~(LC_MASK);  
  #else
    FORCE_DDR &= ~(FORCE_MASK); // Force servo
  #endif

  // Set revision divider pin as an input
  RD_DDR &= ~(RD_MASK);

  // Initialize ADCSRA to 0x00
  ADCSRA = 0x00;

  // Set ADC reference
  ADMUX |= (1 << REFS0);
}

uint16_t adc_read_channel(uint8_t channel) 
{
  // Select Channel - ADCSRB, MUX5
  // If the channel number is higher than 7, the MUX5 bit in ADCSRB needs to be set  
  if (channel & MUX5_MASK) {
    bit_true(ADCSRB, (1 << MUX5_BIT_POS));
  } else {
    bit_false(ADCSRB, (1 << MUX5_BIT_POS));
  }

  // Clear the previously selected channel
  ADMUX &= ~(ADMUX_SELECTION_MASK);
  // Select Channel
  ADMUX |= (channel & ADMUX_SELECTION_MASK);

  // Enable ADC, Prescaler: 128x
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // Enable ADC capture
  ADCSRA |= (1 << ADSC);

  // Enable capture of ADC with timeout
  uint16_t timeout = 0xFFFF;
  while(ADCSRA & (1 << ADSC)) {
    if (timeout-- == 0) {
      return 0;
    }
  }
    
  // Stote the result
  uint16_t ret = ADC;
  
  // Disable ADC 
  ADCSRA &= ~(1<<ADEN);
 
  // Return result
  return ret;
}

