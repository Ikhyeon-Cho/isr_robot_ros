#include "Controller.h"

volatile bool emergencyButtonPressed = false;

bool stopSign = false;  // 모터가 잠겨있거나 속도가 0인 상황에서는 true, 아니면 false: 정지상황에서 시간이 지나면 로봇이
                        // 조금씩 움직이는 상황이 생겨서 그걸 막기위한 변수.

// Singleton structure's instance
ControllerBoard controller;

SIGNAL(PCINT0_vect)
{
  emergencyButtonPressed = digitalRead(PIN_EMERGENCY_STOP_BUTTON);

  if (emergencyButtonPressed)
  {
    controller.MotorSpeed(0, 0);
    delay(1000);
    controller.MotorStop(true);
  }
  else
  {
    controller.MotorStop(false);
  }
}

ControllerBoard::ControllerBoard()
{
#ifndef USE_ROS_SERIAL
  Serial.begin(115200);
#else
  // comm.init();
#endif
}

void ControllerBoard::Initialize()
{
#ifndef USE_ROS_SERIAL
  // initialize communication
  Serial.begin(115200);
  messageIn.begin(&Serial);
  messageOut.begin(&Serial);
#else
  comm.init();
#endif

  // set the digital pins as output for right wheel
  pinMode(PIN_RIGHT_MOTOR_PWM_DIG, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR_EN_DIG, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR_DIR_DIG, OUTPUT);
  pinMode(PIN_RIGHT_MOTOR_STOP_DIG, OUTPUT);

  // set the digital pins as output for left wheel
  pinMode(PIN_LEFT_MOTOR_PWM_DIG, OUTPUT);
  pinMode(PIN_LEFT_MOTOR_EN_DIG, OUTPUT);
  pinMode(PIN_LEFT_MOTOR_DIR_DIG, OUTPUT);
  pinMode(PIN_LEFT_MOTOR_STOP_DIG, OUTPUT);

  // set the analog pins
  pinMode(PIN_RIGHT_MOTOR_ACTUAL_SPEED_AN, INPUT);
  pinMode(PIN_LEFT_MOTOR_ACTUAL_SPEED_AN, INPUT);

  // set pin change interrupt for emergency stop button
  pinMode(PIN_EMERGENCY_STOP_BUTTON, INPUT_PULLUP);
  // EICRA = _BV(ISC30) | _BV(ISC20);
  // EIMSK = _BV(INT3) | _BV(INT2);
  PCICR = _BV(PCIE0);
  PCMSK0 = _BV(PCINT7);
  sei();

  // set up fast PWM on pins (Timer 4):
  TCCR4A = _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41);
  TCCR4B = _BV(CS40) | _BV(WGM43) | _BV(WGM42);
  ICR4 = MOTOR_PWM_TOP;

  // initialize motor
  MotorEnable(true);

  MotorStop(false);
  MotorSpeed(0, 0);
  MotorStop(true);
  stopSign = true;

  encoderCounter.Initialize(4);  // Count Mode   1, 2, 4X
}
int leftMotorRPM = 0, rightMotorRPM = 0;

void ControllerBoard::SpinOnce()
{
  uint8_t codename;
  int value;
  int actual_leftMotorRPM, actual_rightMotorRPM;
  unsigned long leftEncoder, rightEncoder;

  MotorStop(emergencyButtonPressed | motorStopStatus);
  if (!emergencyButtonPressed)
    MotorSpeed(leftMotorRPM, rightMotorRPM);

#ifndef USE_ROS_SERIAL
  PCMSK0 &= 0x7F;  // PCINT7 disable (emergency brake interrupt)
  bool ret_Serial = this->messageIn.receiveData();
  PCMSK0 |= 0x80;  // PCINT7 enable (emergency brake interrupt)
  PCIFR |= 0x01;   // PCIF0 clear
  emergencyButtonPressed = digitalRead(PIN_EMERGENCY_STOP_BUTTON);
  if (ret_Serial)
  {
    // Serial.println("data received");
    uint8_t command = messageIn.readByte();

    switch (command)
    {
      case COMMAND_INITIALIZE:
        this->Initialize();
        encoderCounter.Initialize(4);  // Count Mode   4X
        break;

      case COMMAND_MOTOR_ENABLE:
        motorEnableStatus = messageIn.readByte();
        MotorEnable(motorEnableStatus);
        break;
      case COMMAND_MOTOR_RUN:
        leftMotorRPM = messageIn.readInt();
        rightMotorRPM = messageIn.readInt();
        MotorSpeed(leftMotorRPM, rightMotorRPM);
        break;
      case COMMAND_MOTOR_STOP:
        motorStopStatus = messageIn.readByte();
        MotorStop(motorStopStatus);
        break;
      case COMMAND_MOTOR_ACTUAL_SPEED_READ:
        ReadActualMotorSpeed(actual_leftMotorRPM, actual_rightMotorRPM);
        messageOut.writeByte(COMMAND_MOTOR_ACTUAL_SPEED_READ_RE);  // for return message
        messageOut.writeByte(!leftMotorDirection);
        messageOut.writeByte(rightMotorDirection);
        messageOut.writeInt(actual_leftMotorRPM);
        messageOut.writeInt(actual_rightMotorRPM);
        messageOut.sendData();
        break;

      case COMMAND_ENCODER_READ:
        ReadEncoder(leftEncoder, rightEncoder);
        messageOut.writeByte(COMMAND_ENCODER_READ_RE);  // for return message
        messageOut.writeLong(leftEncoder);
        messageOut.writeLong(rightEncoder);
        messageOut.sendData();
        break;
      case COMMAND_ENCODER_RESET:
        ResetEncoder();
        break;

      case COMMAND_STATUS:
        messageOut.writeByte(COMMAND_STATUS_RE);  // for return message
        messageOut.writeByte(motorEnableStatus);
        messageOut.writeByte(motorStopStatus);
        messageOut.writeByte(emergencyButtonPressed);
        messageOut.sendData();
        break;
    }
  }
#else
  comm.spinOnce();
#endif  // #ifndef USE_ROS_SERIAL
}

void ControllerBoard::MotorEnable(bool value)
{
  motorEnableStatus = value;
  digitalWrite(PIN_RIGHT_MOTOR_EN_DIG, value);
  digitalWrite(PIN_LEFT_MOTOR_EN_DIG, value);
}

void ControllerBoard::MotorSpeed(int leftMotorRPM, int rightMotorRPM)
{
  static unsigned long stopTime = 0;
  if (abs(leftMotorRPM) > MOTOR_MAX_RPM || abs(rightMotorRPM) > MOTOR_MAX_RPM)
  {
    return;
  }

  MotorDirection(leftMotorRPM, rightMotorRPM);

  if (leftMotorRPM == 0 && rightMotorRPM == 0)

  {
    if (!stopSign)
    {
      stopSign = true;
      stopTime = millis();
    }
    if (stopSign && millis() - stopTime < 1000)
    {
      OCR4C = map(abs(rightMotorRPM), 0, MOTOR_MAX_RPM, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
      OCR4B = map(abs(leftMotorRPM), 0, MOTOR_MAX_RPM, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    }
    else
    {
      MotorStop(true);
    }
  }
  else
  {
    if (stopSign)
    {
      stopSign = false;
    }
    MotorStop(false);
    OCR4C = map(abs(rightMotorRPM), 0, MOTOR_MAX_RPM, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
    OCR4B = map(abs(leftMotorRPM), 0, MOTOR_MAX_RPM, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  }
}

void ControllerBoard::MotorDirection(int leftMotorRPM, int rightMotorRPM)
{
  pinMode(PIN_RIGHT_MOTOR_DIR_DIG, OUTPUT);
  if (rightMotorRPM >= 0)
  {
    rightMotorDirection = RIGHT_MOTOR_FORWARD_DIR;
    digitalWrite(PIN_RIGHT_MOTOR_DIR_DIG, RIGHT_MOTOR_FORWARD_DIR);
  }
  else
  {
    rightMotorDirection = RIGHT_MOTOR_BACKWARD_DIR;
    digitalWrite(PIN_RIGHT_MOTOR_DIR_DIG, RIGHT_MOTOR_BACKWARD_DIR);
  }

  pinMode(PIN_LEFT_MOTOR_DIR_DIG, OUTPUT);
  if (leftMotorRPM >= 0)
  {
    leftMotorDirection = LEFT_MOTOR_FORWARD_DIR;
    digitalWrite(PIN_LEFT_MOTOR_DIR_DIG, LEFT_MOTOR_FORWARD_DIR);
  }
  else
  {
    leftMotorDirection = LEFT_MOTOR_BACKWARD_DIR;
    digitalWrite(PIN_LEFT_MOTOR_DIR_DIG, LEFT_MOTOR_BACKWARD_DIR);
  }
}

void ControllerBoard::MotorStop(bool value)
{
  motorStopStatus = value;
  digitalWrite(PIN_RIGHT_MOTOR_STOP_DIG, value);
  digitalWrite(PIN_LEFT_MOTOR_STOP_DIG, value);
}

void ControllerBoard::ReadActualMotorSpeed(int& actual_leftMotorRPM, int& actual_rightMotorRPM)
{
  actual_rightMotorRPM = analogRead(PIN_RIGHT_MOTOR_ACTUAL_SPEED_AN);
  actual_leftMotorRPM = analogRead(PIN_LEFT_MOTOR_ACTUAL_SPEED_AN);
}

void ControllerBoard::ReadEncoder(unsigned long& leftEncoder, unsigned long& rightEncoder)
{
  rightEncoder = encoderCounter.ReadRightEncoderCount();
  leftEncoder = encoderCounter.ReadLeftEncoderCount();
}

void ControllerBoard::ResetEncoder(void)
{
  encoderCounter.ResetRightEncoder();
  encoderCounter.ResetLeftEncoder();
}

bool ControllerBoard::IsMotorEnabled() const
{
  return motorEnableStatus;
}

bool ControllerBoard::IsMotorStopped() const
{
  return motorStopStatus;
}

bool ControllerBoard::IsEStopPressed() const
{
  return emergencyButtonPressed;
}

bool ControllerBoard::IsRightMotorDirForward() const
{
  return rightMotorDirection;
}

bool ControllerBoard::IsLeftMotorDirForward() const
{
  return !leftMotorDirection;
}
