#ifndef SONIC_H_INCLUDED
#define SONIC_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class HC_SR04 {
    public:
        void Init(int trigpin, int echopin, int ctm);
        float ReadLength(void);

    private:
        int TRIGPIN;
        int ECHOPIN;
        int CTM;
};

extern HC_SR04 sonic;
#endif
