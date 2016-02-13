#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Control.h"

class Motor : private Control {
  public:
    void SetPinNum(int motorLF, int motorRF, int motorLB, int motorRB);
    void SetControlLimit(int min, int max);
    void Control(const int motorL, const int motorR);
    void SteerControl(float command, float current);

  private:
    int MotorLF, MotorRF, MotorLB, MotorRB;
    int MinValue = 0, MaxValue = 255;
};

extern Motor motor;

#endif