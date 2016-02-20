#ifndef CONTROL_H_INCLUDED
#define CONTROL_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Control {
  public:
    float getDt(void);
    void SetPIDGain(float pGain, float iGain, float dGain);
    float PIDControl(float dCommand, float dVal);

  private:
    float PGain, IGain, DGain;
};

extern Control control;

#endif
