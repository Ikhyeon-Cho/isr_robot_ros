/*
  ISR-M2 Controller

  PWM signals are output on digital pins 2 to 13.
  This sketch was written for the Arduino Mega, and will not work on previous boards.
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "EasyTransfer2.h"
#include "EncoderCounter.h"
#include "Arduino.h"

class ControllerBoard
{
public:
  ControllerBoard();

public:
  void Initialize();
  void SpinOnce();

  void MotorSpeed(int leftMotorRPM, int rightMotorRPM);

  void MotorEnable(bool value);
  void MotorDirection(int leftMotorRPM, int rightMotorRPM);
  void MotorStop(bool value);
  void ReadActualMotorSpeed(int& actual_leftMotorRPM, int& actual_rightMotorRPM);

  void ResetEncoder(void);  // Reset left & right encoders
  void ReadEncoder(unsigned long& leftEncoder, unsigned long& rightEncoder);

  bool IsMotorEnabled() const;
  bool IsMotorStopped() const;
  bool IsEStopPressed() const;
  bool IsRightMotorDirForward() const;
  bool IsLeftMotorDirForward() const;

private:
  bool motorEnableStatus = false;
  bool motorStopStatus = false;
  bool rightMotorDirection, leftMotorDirection;

  EncoderCounter encoderCounter;

  EasyTransfer2 messageIn;
  EasyTransfer2 messageOut;
};

extern ControllerBoard controller;

/* A Message Structure
Initialize (0 byte):
  byte COMMAND_Initialize;

Enable (1 byte):
  byte COMMAND_MOTOR_ENABLE, byte value;
Run (4 bytes):
  byte COMMAND_MOTOR_RUN, int speedL, int speedR;
Stop (1 byte):
  byte COMMAND_MOTOR_STOP, byte value;
Actual Motor Speed (0 byte):
  byte COMMAND_ACTUAL_MOTOR_SPEED_READ;
Actual Motor Speed _return_ (6 bytes):
  byte COMMAND_ACTUAL_MOTOR_SPEED_READ_RE, byte directionL, byte directionR, int actualSpeedL, int actualSpeedR;

Read Encoder (0 byte):
  byte COMMAND_ENCODER_READ;
Read Encoder _return_ (8 bytes):
  byte COMMAND_ENCODER_READ_RE, unsigned long leftEncoder, unsigned long rightEncoder;
Reset Encoder (0 byte):
  byte COMMAND_ENCODER_RESET;

Status (0 byte):
  byte COMMAND_STATUS;
Status _return_ (3 bytes):
  byte COMMAND_STATUS_RE, byte motorEnableStatus, byte motorStopStatus, byte emergencyButtonPressed;
*/

#define COMMAND_INITIALIZE 0

#define COMMAND_MOTOR_ENABLE 1
#define COMMAND_MOTOR_RUN 2
#define COMMAND_MOTOR_STOP 3
#define COMMAND_MOTOR_ACTUAL_SPEED_READ 4
#define COMMAND_MOTOR_ACTUAL_SPEED_READ_RE 5  // for return message

#define COMMAND_ENCODER_READ 11
#define COMMAND_ENCODER_READ_RE 12  // for return message
#define COMMAND_ENCODER_RESET 13

#define COMMAND_STATUS 21
#define COMMAND_STATUS_RE 22  // for return message

#define MOTOR_MAX_RPM 4650
#define MOTOR_PWM_TOP                                                                                                  \
  46500  // MAX_MOTOR_RPM * 10, 16MHz(no prescale) / MAX_MOTOR_RPM = 344.086Hz (ESCON PWM frequency range: 10Hz ~ 5kHz)
#define MOTOR_PWM_MIN 4650   // MAX_MOTOR_RPM * 1
#define MOTOR_PWM_MAX 41850  // MAX_MOTOR_RPM * 9

/*** Right Wheel - ESCON Connection Info
   *** Digital I/O (ESCON Pin  -  Arduino Pin)
   Pin1(DigIN1) -  Digital Pin8 (PH5/OC4C)  // PWM - Set Value
   Pin2(DigIN2) -  Digital Pin9 (PH6/OC2B)  // Enable
   Pin3(DigIN3) -  Digital Pin10 (PB4/OC2A/PCINT4)  // Direction
   Pin4(DigIN4) -  Digital Pin11 (PB5/OC1A/PCINT5)  // Stop
   Pin5(GND)    -  GND
   Pin6(+5 VDC) -  NC
   *** Analog I/O (ESCON Pin  -  Arduino Pin)
   Pin1(AnIN1+) -  NC
   Pin2(AnIN1-) -  NC
   Pin3(AnIN2+) -  NC
   Pin4(AnIN2-) -  NC
   Pin5(AnOUT1) -  Analog pin0 (PF0/ADC0)  // Actual Motor Speed
   Pin6(AnOUT2) -  NC
   Pin7(GND)    -  GND
 */
#define RIGHT_MOTOR_FORWARD_DIR 1
#define RIGHT_MOTOR_BACKWARD_DIR 0
#define PIN_RIGHT_MOTOR_PWM_DIG 8          // Digital Pin8:  PWM - Set Value
#define PIN_RIGHT_MOTOR_EN_DIG 9           // Digital Pin9:  Enable
#define PIN_RIGHT_MOTOR_DIR_DIG 10         // Digital Pin10: Direction
#define PIN_RIGHT_MOTOR_STOP_DIG 11        // Digital Pin11: Stop
#define PIN_RIGHT_MOTOR_ACTUAL_SPEED_AN 0  // Analog pin0:   Actual Motor Speed

/*** Left Wheel - ESCON Connection Info
   *** Digital I/O (ESCON Pin  -  Arduino Pin)
   Pin1(DigIN1) -  Digital Pin7 (PH4/OC4B)  // PWM - Set Value
   Pin2(DigIN2) -  Digital Pin6 (PH3/OC4A)  // Enable
   Pin3(DigIN3) -  Digital Pin5 (PE3/OC3A/AIN1)  // Direction
   Pin4(DigIN4) -  Digital Pin4 (PG5/OC0B)  // Stop
   Pin5(GND)    -  GND
   Pin6(+5 VDC) -  NC
   *** Analog I/O (ESCON Pin  -  Arduino Pin)
   Pin1(AnIN1+) -  NC
   Pin2(AnIN1-) -  NC
   Pin3(AnIN2+) -  NC
   Pin4(AnIN2-) -  NC
   Pin5(AnOUT1) -  Analog pin1 (PF1/ADC1)  // Actual Motor Speed
   Pin6(AnOUT2) -  NC
   Pin7(GND)    -  GND
 */
#define LEFT_MOTOR_FORWARD_DIR 0
#define LEFT_MOTOR_BACKWARD_DIR 1
#define PIN_LEFT_MOTOR_PWM_DIG 7          // Digital Pin7: PWM - Set Value
#define PIN_LEFT_MOTOR_EN_DIG 6           // Digital Pin6: Enable
#define PIN_LEFT_MOTOR_DIR_DIG 5          // Digital Pin5: Direction
#define PIN_LEFT_MOTOR_STOP_DIG 4         // Digital Pin4: Stop
#define PIN_LEFT_MOTOR_ACTUAL_SPEED_AN 1  // Analog pin1:  Actual Motor Speed

// Emergency Stop Button (using Arduino's internal Pull UP registor)
// Arduino Mega Digital Pin 13 [ATMEL ATMEGA PORTB 7 (OC0A/OC1C/PCINT7)]

#define PIN_EMERGENCY_STOP_BUTTON 13

#endif  // _CONTROLLER_H
