#include "Motor.h"

Motor motor;

void Motor::SetPinNum(int motorLF, int motorLB, int motorRF, int motorRB)
{
  PORTD &= ~(1 << 5);
  MotorLF = motorLF;
  MotorRF = motorRF;
  MotorLB = motorLB;
  MotorRB = motorRB;
  pinMode(MotorLF, OUTPUT);
  pinMode(MotorLB, OUTPUT);
  pinMode(MotorRF, OUTPUT);
  pinMode(MotorRB, OUTPUT);
}

void Motor::SetControlLimit(int min, int max)
{
  MinValue = min;
  MaxValue = max;
}

void Motor::Control(int motorL, int motorR)
{
  int motorl, motorr;

  motorl = sign(motorL) * constrain(abs(motorL), MinValue, MaxValue);
  motorr = sign(motorR) * constrain(abs(motorR), MinValue, MaxValue);

  if (motorl <= 0 && motorr <= 0) {
    analogWrite(MotorLF, 0);
    analogWrite(MotorLB, abs(motorl));
    analogWrite(MotorRF, 0);
    analogWrite(MotorRB, abs(motorr));
  } else if (motorl <= 0 && motorr >= 0) {
    analogWrite(MotorLF, 0);
    analogWrite(MotorLB, abs(motorl));
    analogWrite(MotorRF, motorr);
    analogWrite(MotorRB, 0);
  } else if (motorl >= 0 && motorr <= 0) {
    analogWrite(MotorLF, motorl);
    analogWrite(MotorLB, 0);
    analogWrite(MotorRF, 0);
    analogWrite(MotorRB, abs(motorr));
  } else if (motorl >= 0 && motorr >= 0) {
    analogWrite(MotorLF, motorl);
    analogWrite(MotorLB, 0);
    analogWrite(MotorRF, motorr);
    analogWrite(MotorRB, 0);
  }
}

void Motor::SteerControl(float command, float current)
{
  float ControlValue;

  ControlValue = PIDControl(command, current);
  ControlValue = sign(ControlValue)*constrain(fabs(ControlValue), MinValue, MaxValue);
  Control(127 - ControlValue, 127 + ControlValue);
}

void Motor::RunStraight(unsigned long runtime)
{
  unsigned long startTime = millis();
  unsigned long nowTime = millis();
  float x, y, z;
  
  while((nowTime - startTime) < runtime) {
    GetPhysicalValue_deg(&x, &y, &z);
    SteerControl(0, z);
    nowTime = millis(); 
  }
  Control(0, 0);
}

