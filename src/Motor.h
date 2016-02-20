#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Control.h"
#include "L3GD20.h"

#define sign(n) ((n > 0) - (n < 0))

class Motor : public Control, private L3GD20 {
  public:
    void SetPinNum(int motorLF, int motorRF, int motorLB, int motorRB);
    void SetControlLimit(int min, int max);
    void Control(const int motorL, const int motorR);
    void SteerControl(float command, float current);
    void RunStraight(unsigned long runtime);
    
  private:
    int MotorLF, MotorRF, MotorLB, MotorRB;
    int MinValue = 0, MaxValue = 255;
};

extern Motor motor;

#endif
