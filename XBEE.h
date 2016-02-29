#ifndef XBEE_H_INCLUDED
#define XBEE_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SoftwareSerial.h>

class XBEE : public SoftwareSerial {
    public:
        XBEE(uint8_t receivePin, uint8_t transmitPin);
        void TransferData(float data[], int num);
};

#endif
