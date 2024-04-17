/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/*  Author: Donggeun Cha ( Original Author: Andrew Jalics )*/

#ifndef _ENCODERCOUNTER_h
#define _ENCODERCOUNTER_h

#include "Arduino.h"

class EncoderCounter
{
public:
  EncoderCounter(unsigned char countMode = 4);

  void Initialize(unsigned char countMode);

  void ResetRightEncoder(void);
  void ResetLeftEncoder(void);

  unsigned long ReadRightEncoderCount(void);
  unsigned long ReadLeftEncoderCount(void);

  void SwitchCountMode(unsigned char countMode);

private:
  unsigned long count;
  unsigned char busByte;
};

#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_XY_RL 37
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE 36  // Active LOW
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN1 35
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN2 34
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1 33
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2 32
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTX_RIGHT 31  // Active LOW
#define MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY_LEFT 30   // Active LOW

// Arduino Mega Digital Pin 30 [ATMEL ATMEGA PORTC 7] -> HCTL 2032 Pin 11 RSTY - Active LOW
// Arduino Mega Digital Pin 31 [ATMEL ATMEGA PORTC 6] -> HCTL 2032 Pin 12 RSTX - Active LOW
// Arduino Mega Digital Pin 32 [ATMEL ATMEGA PORTC 5] -> HCTL 2032 Pin 26 SEL2
// Arduino Mega Digital Pin 33 [ATMEL ATMEGA PORTC 4] -> HCTL 2032 Pin  6 SEL1
// Arduino Mega Digital Pin 34 [ATMEL ATMEGA PORTC 3] -> HCTL 2032 Pin  3 EN2
// Arduino Mega Digital Pin 35 [ATMEL ATMEGA PORTC 2] -> HCTL 2032 Pin  2 EN1
// Arduino Mega Digital Pin 36 [ATMEL ATMEGA PORTC 1] -> HCTL 2032 Pin  7 OE   - Active LOW
// Arduino Mega Digital Pin 37 [ATMEL ATMEGA PORTC 0] -> HCTL 2032 Pin 32 X/Y

#define MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA PINA  // Encoder data pin is connected to PORTA

// Arduino Mega Digital Pin 22 [ATMEL ATMEGA PORTA 0] <- HCTL 2032 Pin  1 D0
// Arduino Mega Digital Pin 23 [ATMEL ATMEGA PORTA 1] <- HCTL 2032 Pin 15 D1
// Arduino Mega Digital Pin 24 [ATMEL ATMEGA PORTA 2] <- HCTL 2032 Pin 14 D2
// Arduino Mega Digital Pin 25 [ATMEL ATMEGA PORTA 3] <- HCTL 2032 Pin 13 D3
// Arduino Mega Digital Pin 26 [ATMEL ATMEGA PORTA 4] <- HCTL 2032 Pin 12 D4
// Arduino Mega Digital Pin 27 [ATMEL ATMEGA PORTA 5] <- HCTL 2032 Pin 11 D5
// Arduino Mega Digital Pin 28 [ATMEL ATMEGA PORTA 6] <- HCTL 2032 Pin 10 D6
// Arduino Mega Digital Pin 29 [ATMEL ATMEGA PORTA 7] <- HCTL 2032 Pin  9 D7

// Encoder Count UP/DOWN Info
// Right Encoder CCW UP CW  DOWN
// Left  Encoder CW  UP CCW DOWN

// HCTL-2032 Count Mode Info
// Count Mode Illegal Mode EN1 LOW  EN2 LOW
// Count Mode   4X         EN1 HIGH EN2 LOW
// Count Mode   2X         EN1 LOW  EN2 HIGH
// Count Mode   1X         EN1 HIGH EN2 HIGH

// HCTL-2032 Byte Selected Info
// Byte Selected MSB SEL1  LOW SEL2 HIGH
// Byte Selected 2nd SEL1 HIGH SEL2 HIGH
// Byte Selected 3rd SEL1  LOW SEL2 LOW
// Byte Selected LSB SEL1 HIGH SEL2 LOW

// Encoder Right/Left <- HCTL-2032 X/Y Info
// Right Encoder <- XY LOW  X Axis
// Left Encoder <- XY HIGH Y Axis

// Quadrature Encoder connections  MILE 512-6400 CPT
// Pin 1 NC
// Pin 2 VCC
// Pin 3 GND
// Pin 4 NC
// Pin 5 /A
// Pin 6 A
// Pin 7 /B
// Pin 8 B
// Pin 9 Do not connect
// Pin 10 Do not connect

#endif  // _ENCODERCOUNTER_H
