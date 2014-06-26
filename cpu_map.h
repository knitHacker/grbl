/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2013-2014 Sungeun K. Jeon

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

/* The cpu_map.h file serves as a central pin mapping settings file for different processor
   types, i.e. AVR 328p or AVR Mega 2560. Grbl officially supports the Arduino Uno, but the 
   other supplied pin mappings are supplied by users, so your results may vary. */

// NOTE: This is still a work in progress. We are still centralizing the configurations to
// this file, so your success may vary for other CPUs.

#ifndef cpu_map_h
#define cpu_map_h

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.

#error
  #define GRBL_PLATFORM "Atmega328p"
  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD
  #define X_STEP_BIT      2  // Uno Digital Pin 2
  #define Y_STEP_BIT      3  // Uno Digital Pin 3
  #define Z_STEP_BIT      4  // Uno Digital Pin 4
  #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT   5  // Uno Digital Pin 5
  #define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
  #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
  #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

  // Define homing/hard limit switch input pins and limit interrupt vectors. 
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (pinout).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define Z_LIMIT_BIT	   4 // Uno Digital Pin 12
  #else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
  #endif
  #define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect 
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register
  #define LIMIT_INT_ENABLE LIMIT_MASK
  #define LIMIT_BIT_SHIFT  -1      //shift limit pins 1 left to align w/ step bits.
                                   // won't work with variable spindle, unless we move limits to 10,11,12 and
                                   // put spindle on pwm 9, leave dir on 13.  (then shift=0)

  #define CNC_CONFIGURATION

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR    DDRB
  #define SPINDLE_ENABLE_PORT   PORTB
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.  
    #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
  #else
    #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
  #endif  
  #define SPINDLE_DIRECTION_DDR   DDRB
  #define SPINDLE_DIRECTION_PORT  PORTB
  #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRC
    #define COOLANT_MIST_PORT  PORTC
    #define COOLANT_MIST_BIT   4 // Uno Analog Pin 4
  #endif  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRC
  #define PINOUT_PIN       PINC
  #define PINOUT_PORT      PORTC
  #define PIN_RESET        0  // Uno Analog Pin 0
  #define PIN_FEED_HOLD    1  // Uno Analog Pin 1
  #define PIN_CYCLE_START  2  // Uno Analog Pin 2
  #define PINOUT_INT       PCIE1  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT1_vect
  #define PINOUT_PCMSK     PCMSK1 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))
  
  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5  // Uno Analog Pin 5
  #define PROBE_MASK      (1<<PROBE_BIT)

  
  #ifdef VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    #define TCCRA_REGISTER	 TCCR2A
    #define TCCRB_REGISTER	 TCCR2B
    #define OCR_REGISTER     OCR2A

    #define COMB_BIT	     COM2A1
    #define WAVE0_REGISTER	 WGM20
    #define WAVE1_REGISTER	 WGM21
    #define WAVE2_REGISTER	 WGM22
    #define WAVE3_REGISTER	 WGM23

    // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
    #define SPINDLE_PWM_DDR	  SPINDLE_ENABLE_DDR
    #define SPINDLE_PWM_PORT  SPINDLE_ENABLE_PORT
    #define SPINDLE_PWM_BIT	  SPINDLE_ENABLE_BIT // Shared with SPINDLE_ENABLE.
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------

#ifdef CPU_MAP_ATMEGA2560 // (Arduino Mega 2560) Working @EliteEng

  #define GRBL_PLATFORM "Atmega2560"
  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Increase Buffers to make use of extra SRAM
  //#define RX_BUFFER_SIZE		256
  //#define TX_BUFFER_SIZE		128
  //#define BLOCK_BUFFER_SIZE	36
  //#define LINE_BUFFER_SIZE	100

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR      DDRA
  #define STEP_PORT     PORTA
  #define STEP_PIN      PINA
  #define X_STEP_BIT        0 // MEGA2560 Digital Pin 22
  #define Y_STEP_BIT        1 // MEGA2560 Digital Pin 23
  #define Z_STEP_BIT        2 // MEGA2560 Digital Pin 24
  #define C_STEP_BIT        3 // MEGA2560 Digital Pin 25
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<C_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR      DDRA
  #define DIRECTION_PORT     PORTA
  #define DIRECTION_PIN      PINA
  #define X_DIRECTION_BIT   4 // MEGA2560 Digital Pin 26
  #define Y_DIRECTION_BIT   5 // MEGA2560 Digital Pin 27
  #define Z_DIRECTION_BIT   6 // MEGA2560 Digital Pin 28
  #define C_DIRECTION_BIT   7 // MEGA2560 Digital Pin 29
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<C_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR   DDRJ
  #define STEPPERS_DISABLE_PORT  PORTJ
  #define STEPPERS_DISABLE_BIT   1 // MEGA2560 Digital Pin 14 
  #define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)
//TODO: multiple bits.
  

  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRB
  #define LIMIT_PORT      PORTB
  #define LIMIT_PIN       PINB
  #define X_LIMIT_BIT     3 // MEGA2560 Digital Pin 50
  #define Y_LIMIT_BIT     4 // MEGA2560 Digital Pin 10
  #define Z_LIMIT_BIT     5 // MEGA2560 Digital Pin 11
  #define C_LIMIT_BIT     6 // MEGA2650 Digital Pin 12
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
  #define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<C_LIMIT_BIT)) // All limit bits
  #define LIMIT_INT_ENABLE LIMIT_MASK
  #define LIMIT_BIT_SHIFT 3  // shift right to align with step bits


  #define TIMING_PORT PORTB
  #define TIMING_ENABLE_BIT  7
  #define TIMING_MASK        (1<<TIMING_ENABLE_BIT) // LED

  #define CNC_CONFIGURATION

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR   DDRH
  #define SPINDLE_ENABLE_PORT  PORTH
  #define SPINDLE_ENABLE_BIT   3 // MEGA2560 Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // MEGA2560 Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // MEGA2560 Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRH
    #define COOLANT_MIST_PORT  PORTH
    #define COOLANT_MIST_BIT   6 // MEGA2560 Digital Pin 9
  #endif  

  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRK
  #define PINOUT_PIN       PINK
  #define PINOUT_PORT      PORTK
  #define PIN_RESET        0  // MEGA2560 Analog Pin 8
  #define PIN_FEED_HOLD    1  // MEGA2560 Analog Pin 9
  #define PIN_CYCLE_START  2  // MEGA2560 Analog Pin 10
  #define PINOUT_INT       PCIE2  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT2_vect
  #define PINOUT_PCMSK     PCMSK2 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))

  // Define probe switch input pin.
  #define PROBE_DDR       DDRK
  #define PROBE_PIN       PINK
  #define PROBE_PORT      PORTK
  #define PROBE_BIT       3  // MEGA2560 Analog Pin 11
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Start of PWM & Stepper Enabled Spindle
  #ifdef VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    // Set Timer up to use TIMER2B which is attached to Digital Pin 9
    #define TCCRA_REGISTER		TCCR2A
    #define TCCRB_REGISTER		TCCR2B
    #define OCR_REGISTER		OCR2B

    #define COMB_BIT			COM2B1
    #define WAVE0_REGISTER		WGM20
    #define WAVE1_REGISTER		WGM21
    #define WAVE2_REGISTER		WGM22
    #define WAVE3_REGISTER		WGM23

    #define SPINDLE_PWM_DDR		DDRH
    #define SPINDLE_PWM_PORT    PORTH
    #define SPINDLE_PWM_BIT		6 // MEGA2560 Digital Pin 9
  #endif // End of VARIABLE_SPINDLE

#endif

//----------------------------------------------------------------------------------------


#ifdef CPU_MAP_KEYME_2560 // (KeyMe Mega 2560) 

  #define GRBL_PLATFORM "KeyMe 2560"
  // Serial port pins
  #define SERIAL_RX USART0_RX_vect
  #define SERIAL_UDRE USART0_UDRE_vect

  // Increase Buffers to make use of extra SRAM
  //#define RX_BUFFER_SIZE		256
  //#define TX_BUFFER_SIZE		128
  //#define BLOCK_BUFFER_SIZE	36
  //#define LINE_BUFFER_SIZE	100

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR      DDRH
  #define STEP_PORT     PORTH
  #define STEP_PIN      PINH
  #define X_STEP_BIT        0 // Atmega2560 pin 12 / Arduino Digital Pin 17
  #define Y_STEP_BIT        1 // Atmega2560 pin 13 / Arduino Digital Pin 16
  #define Z_STEP_BIT        2 // Atmega2560 pin 14 / Arduino Digital Pin xx
  #define C_STEP_BIT        3 // Atmega2560 pin 15 / Arduino Digital Pin 6
  #define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)|(1<<C_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR      DDRH
  #define DIRECTION_PORT     PORTH
  #define DIRECTION_PIN      PINH
  #define X_DIRECTION_BIT   4 // Atmega2560 pin 16 / Arduino Digital Pin 7
  #define Y_DIRECTION_BIT   5 // Atmega2560 pin 17 / Arduino Digital Pin 8
  #define Z_DIRECTION_BIT   6 // Atmega2560 pin 18 / Arduino Digital Pin 9
  #define C_DIRECTION_BIT   7 // Atmega2560 pin 27 / Arduino Digital Pin xx
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)|(1<<C_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR   DDRJ
  #define STEPPERS_DISABLE_PORT  PORTJ
//  #define STEPPERS_DISABLE_BIT   1 // Atmega2560 pin X / Arduino Digital Pin 14 
//  #define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)
#define STEPPERS_DISABLE_MASK (0x78)   //Atmega2560 pins 66-69.   //Arduino xxxx
//TODO: separate bits if needed
  

  // NOTE: All limit bit pins must be on the same port
  #define LIMIT_DDR       DDRD
  #define LIMIT_PORT      PORTD
  #define LIMIT_PIN       PIND
  #define X_LIMIT_BIT     0 // Atmega2560 pin 43 / Arduino Digital Pin 21
  #define Y_LIMIT_BIT     1 // Atmega2560 pin 44 / Arduino Digital Pin 20
  #define Z_LIMIT_BIT     2 // Atmega2560 pin 45 / Arduino Digital Pin 19
  #define C_LIMIT_BIT     3 // Atmega2560 pin 46 / Arduino Digital Pin 18
  #define LIMIT_INT       PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect  PCINT0_vect 
  #define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
//#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)|(1<<C_LIMIT_BIT)) // All limit bits
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT))
  #define LIMIT_INT_ENABLE LIMIT_MASK
#define LIMIT_BIT_SHIFT   0

  /* #define TIMING_PORT PORTA */
  /* #define TIMING_ENABLE_BIT  7 // Atmega2560 pin 71 / Arduino Digital Pin 29 */
  /* #define TIMING_MASK        (1<<TIMING_ENABLE_BIT) // LED */
#define TIMING_DDR DDRB
#define TIMING_PORT PORTB  ///TODO: move later
  #define TIMING_BIT  7
  #define TIMING_MASK        (1<<TIMING_BIT) // LED


#ifdef CNC_CONFIGURATION
  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR   DDRH
  #define SPINDLE_ENABLE_PORT  PORTH
  #define SPINDLE_ENABLE_BIT   3 // Atmega2560 pin X / Arduino Digital Pin 6
  #define SPINDLE_DIRECTION_DDR   DDRE
  #define SPINDLE_DIRECTION_PORT  PORTE
  #define SPINDLE_DIRECTION_BIT   3 // Atmega2560 pin X / Arduino Digital Pin 5

  // Define flood and mist coolant enable output pins.
  // NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
  // a later date if flash and memory space allows.
  #define COOLANT_FLOOD_DDR   DDRH
  #define COOLANT_FLOOD_PORT  PORTH
  #define COOLANT_FLOOD_BIT   5 // Atmega2560 pin X / Arduino Digital Pin 8
  #ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
    #define COOLANT_MIST_DDR   DDRH
    #define COOLANT_MIST_PORT  PORTH
    #define COOLANT_MIST_BIT   6 // Atmega2560 pin X / Arduino Digital Pin 9
  #endif  
#endif


  // Define user-control pinouts (cycle start, reset, feed hold) input pins.
  // NOTE: All pinouts pins must be on the same port and not on a port with other input pins (limits).
  #define PINOUT_DDR       DDRK
  #define PINOUT_PIN       PINK
  #define PINOUT_PORT      PORTK
  #define PIN_RESET        0  // MEGA2560 Analog Pin 8
  #define PIN_FEED_HOLD    1  // MEGA2560 Analog Pin 9
  #define PIN_CYCLE_START  2  // MEGA2560 Analog Pin 10
  #define PINOUT_INT       PCIE2  // Pin change interrupt enable pin
  #define PINOUT_INT_vect  PCINT2_vect
  #define PINOUT_PCMSK     PCMSK2 // Pin change interrupt register
  #define PINOUT_MASK ((1<<PIN_RESET)|(1<<PIN_FEED_HOLD)|(1<<PIN_CYCLE_START))

#endif 
  // Define probe switch input pin.  //MAG HOME
  #define PROBE_DDR       DDRK
  #define PROBE_PIN       PINK
  #define PROBE_PORT      PORTK
  #define PROBE_BIT       3   // Atmega2560 pin 86 // Arduino Analog Pin 11
  #define PROBE_MASK      (1<<PROBE_BIT)


  // Start of PWM & Stepper Enabled Spindle
  #ifdef VARIABLE_SPINDLE
    // Advanced Configuration Below You should not need to touch these variables
    // Set Timer up to use TIMER2B which is attached to Digital Pin 9
    #define TCCRA_REGISTER		TCCR2A
    #define TCCRB_REGISTER		TCCR2B
    #define OCR_REGISTER		OCR2B

    #define COMB_BIT			COM2B1
    #define WAVE0_REGISTER		WGM20
    #define WAVE1_REGISTER		WGM21
    #define WAVE2_REGISTER		WGM22
    #define WAVE3_REGISTER		WGM23

    #define SPINDLE_PWM_DDR		DDRH
    #define SPINDLE_PWM_PORT    PORTH
    #define SPINDLE_PWM_BIT		6 // Atmega2560 pin X / Arduino Digital Pin 9
  #endif // End of VARIABLE_SPINDLE




#endif


/* 
#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and paste one of the default cpu map
  // settings above and modify it to your needs. Then, make sure the defined name is also
  // changed in the config.h file.
#endif
*/

