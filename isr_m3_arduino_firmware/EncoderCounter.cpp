/* Free Software (Beer and Speech) */
/* Please give credit where credit is due. */
/* No stupid legalize, no warranty expressed or implied. */
/* This software is for those who love to create instead of bicker and hamper innovation. */
/* Edited: Donggeun Cha [donggeun.cha@gmail.com]( Author: Andrew Jalics )*/

/* Please see header file (EncoderCounter.h) and Avago HCTL-2032 Datasheet for more technical details */

#include "EncoderCounter.h"

EncoderCounter::EncoderCounter(unsigned char countMode)
{
  Initialize(countMode);
}

void EncoderCounter::Initialize(unsigned char countMode)
{
  // Initialize encoder counter
  DDRA = 0x00;   // sets Arduino Mega (ATMEL ATMEGA) Digital pins 22(PORTA0) to 29(PORTA7) as inputs from HCTL-2032 - D0 to D7

  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_XY_RL,      OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,         OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN1,        OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN2,        OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1,       OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2,       OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTX_RIGHT, OUTPUT);
  pinMode(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY_LEFT,  OUTPUT);

  // XY LOW  X Axis AKA 1st Axis
  // XY HIGH Y Axis AKA 2nd Axis
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_XY_RL, LOW);

  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE, HIGH);  // Active LOW

  SwitchCountMode(countMode);

  // Byte Selected MSB SEL1  LOW SEL2 HIGH
  // Byte Selected 2nd SEL1 HIGH SEL2 HIGH
  // Byte Selected 3rd SEL1  LOW SEL2 LOW
  // Byte Selected LSB SEL1 HIGH SEL2 LOW
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);

  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTX_RIGHT, HIGH);  // Active LOW
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY_LEFT,  HIGH);  // Active LOW
  
  ResetRightEncoder( );
  ResetLeftEncoder( );
}

// Communicates with a HCTL-2032 IC to get reset the right wheel encoder count
// see Avago/Agilent/HP HCTL-2032 PDF for details
void EncoderCounter::ResetRightEncoder( )
{
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTX_RIGHT, LOW);
  delayMicroseconds(1);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTX_RIGHT, HIGH);
  delayMicroseconds(1);
}

// Communicates with a HCTL-2032 IC to get the right wheel encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2032 PDF for details
unsigned long EncoderCounter::ReadRightEncoderCount( )
{
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_XY_RL,   LOW);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,      LOW);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1,    LOW);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2,    HIGH);
  delayMicroseconds(1);
  busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
  count   = busByte;
  count <<= 8;

  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
  delayMicroseconds(1);
  busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
  count  += busByte;
  count <<= 8;

  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
  delayMicroseconds(1);
  busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
  count  += busByte;
  count <<= 8;

  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
  delayMicroseconds(1);
  busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
  count  += busByte;

  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,  HIGH);

  return count;
}

// Communicates with a HCTL-2032 IC to get reset the left wheel encoder count
// see Avago/Agilent/HP HCTL-2032 PDF for details
void EncoderCounter::ResetLeftEncoder( )
{
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY_LEFT, LOW);
  delayMicroseconds(1);
  digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_RSTY_LEFT, HIGH);
  delayMicroseconds(1);
}

// Communicates with a HCTL-2032 IC to get the left wheel encoder count via an 8bit parallel bus
// see Avago/Agilent/HP HCTL-2032 Datasheet PDF for details
unsigned long EncoderCounter::ReadLeftEncoderCount( )
{
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_XY_RL,   HIGH);
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,      LOW);
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1,    LOW);
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2,    HIGH);
   delayMicroseconds(1);
   busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
   count   = busByte;
   count <<= 8;

   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, HIGH);
   delayMicroseconds(1);
   busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
   count  += busByte;
   count <<= 8;

   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, LOW);
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
   count  += busByte;
   count <<= 8;

   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL1, HIGH);
   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_SEL2, LOW);
   delayMicroseconds(1);
   busByte = MEGA_QUADRATURE_ENCODER_COUNTER_PORT_DATA;
   count  += busByte;

   digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_OE,  HIGH);

   return count;
}

// Communicates with a HCTL-2032 IC to set the count mode
// see Avago/Agilent/HP HCTL-2032 PDF for details
void EncoderCounter::SwitchCountMode(unsigned char countMode)
{
   // Count Mode Illegal Mode EN1 LOW  EN2 LOW
   // Count Mode   4X         EN1 HIGH EN2 LOW
   // Count Mode   2X         EN1 LOW  EN2 HIGH
   // Count Mode   1X         EN1 HIGH EN2 HIGH
   switch(countMode)
   {
      case 1: // 1X Count Mode
         digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN1, HIGH);
         digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN2, HIGH);
         break;
      case 2: // 2X Count Mode
         digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN1, LOW);
         digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN2, HIGH);
         break;
      case 4: // 4X Count Mode is the default
      default:
         digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN1, HIGH);
         digitalWrite(MEGA_QUADRATURE_ENCODER_COUNTER_PIN_EN2, LOW);
         break;
   }
   delayMicroseconds(1);
}

