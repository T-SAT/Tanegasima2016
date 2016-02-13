#include "Motor.h"

Motor motor;

void Motor::SetPinNum(int motorLF, int motorRF, int motorLB, int motorRB)
{
  MotorLF = motorLF;
  MotorRF = motorRF;
  MotorLB = motorLB;
  MotorRB = motorRB;
}

void Motor::SetControlLimit(int min, int max)
{
  MinValue = min;
  MaxValue = max;
}

void Motor::Control(int motorL, int motorR)
{

}

void Motor::SteerControl(float command, float current)
{
  float ControlValue;

  ControlValue = PIDControl(command, current);
  if (ControlValue < 0) {
    ControlValue = -ControlValue;
    ControlValue = constrain(ControlValue, 1, 255);
    Control(255, 255 - ControlValue);
  }
  else {
    ControlValue = constrain(ControlValue, 1, 255);
    Control(255 - ControlValue, 255);
  }
}

